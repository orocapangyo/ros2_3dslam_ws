#include "mapper/wall_aligner_node.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <cmath>
#include <random>
#include <thread>
#include <chrono>

namespace mapper {

WallAlignerNode::WallAlignerNode(const rclcpp::NodeOptions & options)
: Node("wall_aligner_node", options)
{
    using namespace std::placeholders;

    // /scan QoS: BEST_EFFORT + VOLATILE (LiDAR 센서 표준)
    auto scan_qos = rclcpp::QoS(10)
        .best_effort()
        .durability_volatile();
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", scan_qos,
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(scan_mutex_);
            latest_scan_ = msg;
        });

    spin_client_ = rclcpp_action::create_client<SpinAction>(this, "spin");

    auto cb_group = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    wall_align_server_ = rclcpp_action::create_server<WallAlignAction>(
        this, "wall_align",
        std::bind(&WallAlignerNode::handle_goal, this, _1, _2),
        std::bind(&WallAlignerNode::handle_cancel, this, _1),
        std::bind(&WallAlignerNode::handle_accepted, this, _1),
        rcl_action_server_get_default_options(), cb_group);
}

Line2D WallAlignerNode::detect_longest_wall(
    const sensor_msgs::msg::LaserScan & scan)
{
    std::vector<std::pair<double,double>> pts;
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float r = scan.ranges[i];
        if (r < scan.range_min || r > scan.range_max || !std::isfinite(r)) continue;
        double angle = scan.angle_min + i * scan.angle_increment;
        pts.push_back({r * std::cos(angle), r * std::sin(angle)});
    }

    if (pts.size() < 2) return {0.0, 0};

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> dist(0, (int)pts.size() - 1);
    const int ITERATIONS = 100;
    const double INLIER_DIST = 0.05;

    Line2D best{0.0, 0};
    for (int iter = 0; iter < ITERATIONS; ++iter) {
        int i1 = dist(rng), i2 = dist(rng);
        if (i1 == i2) continue;
        double dx = pts[i2].first  - pts[i1].first;
        double dy = pts[i2].second - pts[i1].second;
        double len = std::hypot(dx, dy);
        if (len < 0.1) continue;

        double a = dy / len, b = -dx / len;
        double c = -(a * pts[i1].first + b * pts[i1].second);

        int inliers = 0;
        for (auto & p : pts) {
            if (std::abs(a * p.first + b * p.second + c) < INLIER_DIST) {
                ++inliers;
            }
        }
        if (inliers > best.inlier_count) {
            double raw_angle = std::atan2(dy, dx) * 180.0 / M_PI;
            double normalized = std::fmod(raw_angle + 180.0, 180.0);
            best = {normalized, inliers};
        }
    }
    return best;
}

double WallAlignerNode::get_robot_yaw_deg()
{
    // TF2를 사용해 map->base_link yaw 반환
    // 단순화: 0.0 반환 (실제 사용 시 TF lookup 구현)
    return 0.0;
}

rclcpp_action::GoalResponse WallAlignerNode::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const WallAlignAction::Goal> goal)
{
    if (goal->tolerance_deg <= 0.0) {
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WallAlignerNode::handle_cancel(
    const std::shared_ptr<GoalHandleWallAlign>)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void WallAlignerNode::handle_accepted(
    const std::shared_ptr<GoalHandleWallAlign> goal_handle)
{
    std::thread([this, goal_handle]() {
        execute(goal_handle);
    }).detach();
}

void WallAlignerNode::execute(
    const std::shared_ptr<GoalHandleWallAlign> goal_handle)
{
    auto goal     = goal_handle->get_goal();
    auto feedback = std::make_shared<WallAlignAction::Feedback>();
    auto result   = std::make_shared<WallAlignAction::Result>();

    std::atomic<bool> cancelled{false};

    for (int attempt = 0; attempt < 5; ++attempt) {
        if (goal_handle->is_canceling()) {
            result->status = -3;
            goal_handle->canceled(result);
            return;
        }

        sensor_msgs::msg::LaserScan scan_copy;
        {
            std::lock_guard<std::mutex> lock(scan_mutex_);
            if (!latest_scan_) {
                RCLCPP_WARN(get_logger(), "No scan received yet, waiting...");
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }
            scan_copy = *latest_scan_;
        }

        Line2D wall = detect_longest_wall(scan_copy);
        double robot_yaw = get_robot_yaw_deg();
        double error = robot_yaw - wall.angle_deg;
        while (error >  90.0) error -= 180.0;
        while (error < -90.0) error += 180.0;

        feedback->current_error_deg = error;
        feedback->wall_angle_deg    = wall.angle_deg;
        goal_handle->publish_feedback(feedback);

        if (std::abs(error) <= goal->tolerance_deg) {
            result->aligned_heading      = robot_yaw;
            result->status               = 0;
            result->current_error_deg    = error;
            result->wall_angle_deg       = wall.angle_deg;
            goal_handle->succeed(result);
            return;
        }

        double target = robot_yaw - error;
        if (!send_spin_and_wait(target, cancelled)) {
            result->status = cancelled.load() ? -3 : -1;
            if (cancelled.load()) goal_handle->canceled(result);
            else                  goal_handle->abort(result);
            return;
        }
    }

    result->status = -1;
    goal_handle->abort(result);
}

bool WallAlignerNode::send_spin_and_wait(
    double target_angle_deg,
    std::atomic<bool> & cancelled)
{
    if (!spin_client_->wait_for_action_server(std::chrono::seconds(3))) {
        RCLCPP_ERROR(get_logger(), "SpinAction server not available");
        return false;
    }

    auto goal = SpinAction::Goal{};
    goal.target_angle         = target_angle_deg;
    goal.max_angular_speed    = spin_speed_deg_s_;
    goal.angular_acceleration = spin_accel_deg_s2_;
    goal.hold_steer           = false;
    goal.exit_steer_angle     = 0.0;

    std::atomic<bool> done{false};
    std::atomic<bool> success{false};

    auto opts = rclcpp_action::Client<SpinAction>::SendGoalOptions{};
    opts.result_callback =
        [&done, &success, &cancelled](const auto & wrapped) {
            // SpinAction: status 0=success, -1=cancelled
            success.store(wrapped.result->status == 0);
            if (wrapped.result->status == -1) cancelled.store(true);
            done.store(true);
        };

    spin_client_->async_send_goal(goal, opts);
    while (!done.load() && rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return success.load();
}

}  // namespace mapper
