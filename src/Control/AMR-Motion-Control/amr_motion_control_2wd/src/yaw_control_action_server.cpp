#include "amr_motion_control_2wd/yaw_control_action_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "amr_motion_control_2wd/motion_common.hpp"
#include "amr_motion_control_2wd/motion_profile.hpp"
#include "amr_motion_control_2wd/localization_watchdog.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace amr_motion_control_2wd
{


YawControlActionServer::YawControlActionServer(rclcpp::Node::SharedPtr node)
: node_(node)
{
  control_rate_hz_          = safeParam(node_, "yaw_ctrl.control_rate_hz",    20.0);
  min_vx_                   = safeParam(node_, "yaw_ctrl.min_vx",              0.02);
  goal_reach_threshold_     = safeParam(node_, "yaw_ctrl.goal_reach_threshold", 0.05);
  max_timeout_sec_          = safeParam(node_, "yaw_ctrl.max_timeout_sec",     60.0);
  Kp_heading_               = safeParam(node_, "yaw_ctrl.Kp_heading",           1.0);
  Kd_heading_               = safeParam(node_, "yaw_ctrl.Kd_heading",           0.3);
  max_omega_                = safeParam(node_, "yaw_ctrl.max_omega",             1.0);
  min_turning_radius_       = safeParam(node_, "yaw_ctrl.min_turning_radius",    0.7);
  omega_rate_limit_         = safeParam(node_, "yaw_ctrl.omega_rate_limit",      0.5);
  walk_accel_limit_         = safeParam(node_, "yaw_ctrl.walk_accel_limit",      0.5);
  walk_decel_limit_         = safeParam(node_, "yaw_ctrl.walk_decel_limit",      1.0);
  robot_base_frame_         = safeParam(node_, "robot_base_frame", std::string("base_footprint"));

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  cmd_vel_pub_  = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  path_viz_pub_ = node_->create_publisher<nav_msgs::msg::Path>("yaw_ctrl_path_viz", 1);

  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::SensorDataQoS(),
    std::bind(&YawControlActionServer::imuCallback, this, _1));

  std::string pose_topic = safeParam(node_, "yaw_control_pose_topic", std::string("/rtabmap/odom"));
  pose_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    pose_topic, rclcpp::SensorDataQoS(),
    std::bind(&YawControlActionServer::poseCallback, this, _1));

  action_server_ = rclcpp_action::create_server<YawControl>(
    node_, "amr_motion_yaw_control",
    std::bind(&YawControlActionServer::handle_goal,     this, _1, _2),
    std::bind(&YawControlActionServer::handle_cancel,   this, _1),
    std::bind(&YawControlActionServer::handle_accepted, this, _1));

  RCLCPP_INFO(node_->get_logger(), "YawControlActionServer ready");
}

void YawControlActionServer::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                    msg->orientation.z, msg->orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  last_yaw_rad_.store(yaw);
  imu_received_.store(true);
}

void YawControlActionServer::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  if (watchdog_) { watchdog_->updatePose(x, y, yaw); }
}

bool YawControlActionServer::lookupRobotPose(double & x, double & y, double & yaw) const
{
  try {
    auto tf = tf_buffer_->lookupTransform("map", robot_base_frame_,
                                          tf2::TimePointZero,
                                          tf2::durationFromSec(0.1));
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                      tf.transform.rotation.z, tf.transform.rotation.w);
    double roll, pitch;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "YawControlActionServer: TF lookup failed: %s", ex.what());
    return false;
  }
}

void YawControlActionServer::publishCmdVel(double vx, double omega)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x  = vx;
  msg.angular.z = omega;
  cmd_vel_pub_->publish(msg);
}

rclcpp_action::GoalResponse YawControlActionServer::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const YawControl::Goal> /*goal*/)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse YawControlActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleYaw> /*goal_handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void YawControlActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleYaw> goal_handle)
{
  std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

void YawControlActionServer::execute(const std::shared_ptr<GoalHandleYaw> goal_handle)
{
  // ── Mutual exclusion: only one action at a time ───────────────────────────
  ActiveAction expected = ActiveAction::NONE;
  if (!g_active_action.compare_exchange_strong(expected, ActiveAction::YAW_CONTROL)) {
    RCLCPP_WARN(node_->get_logger(),
      "YawControlActionServer: another action is active (%s), aborting new goal",
      to_string(g_active_action.load()));
    auto result = std::make_shared<YawControl::Result>();
    result->status = -2;  // invalid_param (busy)
    try { goal_handle->abort(result); } catch (const std::exception & e) {
      RCLCPP_WARN(node_->get_logger(),
        "YawControlActionServer: abort failed (stale handle): %s", e.what());
    }
    return;
  }
  ActionGuard action_guard;  // clears g_active_action on scope exit

  const auto & goal   = goal_handle->get_goal();
  const auto start_time = node_->now();

  // ── 1. Validate path ─────────────────────────────────────────────────────
  const double dx       = goal->end_x - goal->start_x;
  const double dy       = goal->end_y - goal->start_y;
  const double path_len = std::hypot(dx, dy);

  if (path_len < 1e-4) {
    RCLCPP_WARN(node_->get_logger(),
      "YawControlActionServer: path length near zero (%.4f m), aborting", path_len);
    auto result = std::make_shared<YawControl::Result>();
    result->status      = -2;  // invalid_param
    result->elapsed_time = 0.0;
    try { goal_handle->abort(result); } catch (const std::exception & e) {
      RCLCPP_WARN(node_->get_logger(),
        "YawControlActionServer: abort failed (stale handle): %s", e.what());
    }
    return;
  }

  if (goal->max_linear_speed <= 0.0 || goal->acceleration <= 0.0) {
    RCLCPP_WARN(node_->get_logger(),
      "YawControlActionServer: invalid params (speed=%.3f accel=%.3f), aborting",
      goal->max_linear_speed, goal->acceleration);
    auto result = std::make_shared<YawControl::Result>();
    result->status      = -2;  // invalid_param
    result->elapsed_time = 0.0;
    try { goal_handle->abort(result); } catch (const std::exception & e) {
      RCLCPP_WARN(node_->get_logger(),
        "YawControlActionServer: abort failed (stale handle): %s", e.what());
    }
    return;
  }

  // ── 2. Compute path geometry ──────────────────────────────────────────────
  const double theta_path      = std::atan2(dy, dx);
  const double target_distance = path_len;
  const double ux              = dx / path_len;
  const double uy              = dy / path_len;

  RCLCPP_INFO(node_->get_logger(),
    "YawControlActionServer: goal (%.3f,%.3f)->(%.3f,%.3f) "
    "theta=%.3f rad len=%.3f m v=%.3f a=%.3f",
    goal->start_x, goal->start_y, goal->end_x, goal->end_y,
    theta_path, target_distance, goal->max_linear_speed, goal->acceleration);

  // ── 3. Trapezoidal speed profile ──────────────────────────────────────────
  amr_motion_control::TrapezoidalProfile profile(
    target_distance,
    goal->max_linear_speed,
    goal->acceleration,
    0.0);  // exit speed = 0 (stop at goal)

  // ── 4. Localization watchdog ──────────────────────────────────────────────
  LocalizationWatchdog::Config wd_cfg;
  wd_cfg.timeout_sec          = 2.0;
  wd_cfg.fixed_jump_threshold = 0.5;
  wd_cfg.velocity_margin      = 1.3;
  watchdog_.emplace(wd_cfg, node_->get_logger());

  // ── 5. Wait for pose (5 s timeout) ───────────────────────────────────────
  {
    rclcpp::Rate wait_rate(20);
    int wait_count = 0;
    while (rclcpp::ok() && !watchdog_->poseReceived()) {
      if (!rclcpp::ok()) break;
      wait_rate.sleep();
      if (++wait_count > 100) {
        RCLCPP_ERROR(node_->get_logger(),
          "YawControlActionServer: no pose received after 5 s, aborting");
        publishCmdVel(0.0, 0.0);
        auto result = std::make_shared<YawControl::Result>();
        result->status       = -3;  // timeout
        result->elapsed_time = (node_->now() - start_time).seconds();
        try { goal_handle->abort(result); } catch (const std::exception & e) {
          RCLCPP_WARN(node_->get_logger(),
            "YawControlActionServer: abort failed (stale handle): %s", e.what());
        }
        return;
      }
    }
  }

  // ── 6. Get initial robot pose ─────────────────────────────────────────────
  double rob_x = 0.0, rob_y = 0.0, rob_yaw = 0.0;
  if (!lookupRobotPose(rob_x, rob_y, rob_yaw)) {
    rob_x   = watchdog_->x();
    rob_y   = watchdog_->y();
    rob_yaw = watchdog_->yaw();
  }
  RCLCPP_INFO(node_->get_logger(),
    "YawControlActionServer: initial pose=(%.3f, %.3f, %.3f rad) "
    "path_start=(%.3f,%.3f) arrive_thresh=%.3f m",
    rob_x, rob_y, rob_yaw,
    goal->start_x, goal->start_y, goal_reach_threshold_);

  // Reference origin: path start (map frame)
  const double ref_x = goal->start_x;
  const double ref_y = goal->start_y;

  // ── 7. Control loop ───────────────────────────────────────────────────────
  rclcpp::Rate rate(control_rate_hz_);
  const double dt = 1.0 / control_rate_hz_;

  double prev_e_theta = 0.0;
  double prev_omega   = 0.0;
  double prev_vx      = 0.0;

  while (rclcpp::ok()) {
    if (!rclcpp::ok()) break;

    // ── a. Cancellation check ─────────────────────────────────────────────
    if (goal_handle->is_canceling()) {
      publishCmdVel(0.0, 0.0);
      auto result = std::make_shared<YawControl::Result>();
      result->status          = -1;  // cancelled
      result->actual_distance = 0.0;
      result->elapsed_time    = (node_->now() - start_time).seconds();
      try { goal_handle->canceled(result); } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(),
          "YawControlActionServer: canceled failed (stale handle): %s", e.what());
      }
      RCLCPP_INFO(node_->get_logger(), "YawControlActionServer: cancelled");
      return;
    }

    // ── a. Timeout check ─────────────────────────────────────────────────
    const double elapsed = (node_->now() - start_time).seconds();
    if (elapsed > max_timeout_sec_) {
      publishCmdVel(0.0, 0.0);
      RCLCPP_ERROR(node_->get_logger(),
        "YawControlActionServer: timeout after %.1f s, aborting", elapsed);
      auto result = std::make_shared<YawControl::Result>();
      result->status       = -3;  // timeout
      result->elapsed_time = elapsed;
      try { goal_handle->abort(result); } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(),
          "YawControlActionServer: abort failed (stale handle): %s", e.what());
      }
      return;
    }

    // ── a. Localization watchdog health check ─────────────────────────────
    if (enable_localization_watchdog_ && !watchdog_->checkHealth()) {
      publishCmdVel(0.0, 0.0);
      RCLCPP_ERROR(node_->get_logger(),
        "YawControlActionServer: localization watchdog triggered, aborting");
      auto result = std::make_shared<YawControl::Result>();
      result->status       = -3;  // timeout (localization lost)
      result->elapsed_time = elapsed;
      try { goal_handle->abort(result); } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(),
          "YawControlActionServer: abort failed (stale handle): %s", e.what());
      }
      return;
    }

    // ── b. Get robot pose (TF2: map→base_footprint) ───────────────────────
    if (!lookupRobotPose(rob_x, rob_y, rob_yaw)) {
      rob_x   = watchdog_->x();
      rob_y   = watchdog_->y();
      rob_yaw = watchdog_->yaw();
    }

    // ── c. Projection along path ──────────────────────────────────────────
    const double dx_r    = rob_x - ref_x;
    const double dy_r    = rob_y - ref_y;
    const double traveled = std::max(0.0, dx_r * ux + dy_r * uy);

    // ── d. Remaining distance ─────────────────────────────────────────────
    const double remaining = target_distance - traveled;

    // ── e. Arrival check ──────────────────────────────────────────────────
    if (remaining <= goal_reach_threshold_) {
      publishCmdVel(0.0, 0.0);
      const double e_lateral = -(dx_r * uy - dy_r * ux);
      const double e_heading_deg =
        normalizeAngle(rob_yaw - theta_path) * 180.0 / M_PI;
      auto result = std::make_shared<YawControl::Result>();
      result->status              = 0;  // success
      result->actual_distance     = traveled;
      result->final_lateral_error = e_lateral;
      result->final_heading_error = e_heading_deg;
      result->elapsed_time        = (node_->now() - start_time).seconds();
      try { goal_handle->succeed(result); } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(),
          "YawControlActionServer: succeed failed (stale handle): %s", e.what());
      }
      RCLCPP_INFO(node_->get_logger(),
        "YawControlActionServer: arrived (dist=%.3f m, lat=%.3f m, "
        "hdg=%.2f deg, t=%.2f s)",
        traveled, e_lateral, e_heading_deg, result->elapsed_time);
      return;
    }

    // ── f. Speed from trapezoidal profile ─────────────────────────────────
    const auto profile_out = profile.getSpeed(std::min(traveled, target_distance));
    double vx = profile_out.speed;

    // ── g. Minimum start speed to overcome static friction ────────────────
    if (profile_out.phase != amr_motion_control::ProfilePhase::DONE &&
        vx < min_vx_ && remaining > goal_reach_threshold_) {
      vx = min_vx_;
    }

    // ── h. Heading error: e_theta = normalizeAngle(rob_yaw - theta_path) ──
    const double e_theta = normalizeAngle(rob_yaw - theta_path);

    // ── i. PD heading control ─────────────────────────────────────────────
    const double de_theta = (e_theta - prev_e_theta) / dt;
    double omega = -Kp_heading_ * e_theta - Kd_heading_ * de_theta;
    prev_e_theta = e_theta;

    // ── j. Clamp omega ────────────────────────────────────────────────────
    // Hard clamp to max_omega_
    omega = std::clamp(omega, -max_omega_, max_omega_);
    // Min turning radius constraint: |omega| <= vx / min_turning_radius_
    if (vx > 1e-6 && min_turning_radius_ > 1e-6) {
      const double omega_radius_limit = vx / min_turning_radius_;
      omega = std::clamp(omega, -omega_radius_limit, omega_radius_limit);
    }

    // ── k. Omega rate limit ───────────────────────────────────────────────
    const double omega_delta_max = omega_rate_limit_ * dt;
    omega = std::clamp(omega,
                       prev_omega - omega_delta_max,
                       prev_omega + omega_delta_max);
    prev_omega = omega;

    // ── l. Walk velocity smoother (accel/decel limit on vx) ───────────────
    const double vx_accel_step = walk_accel_limit_ * dt;
    const double vx_decel_step = walk_decel_limit_ * dt;
    if (vx > prev_vx) {
      vx = std::min(vx, prev_vx + vx_accel_step);
    } else {
      vx = std::max(vx, prev_vx - vx_decel_step);
    }
    prev_vx = vx;

    // ── m. Publish velocity ───────────────────────────────────────────────
    watchdog_->setCurrentSpeed(std::abs(vx));
    publishCmdVel(vx, omega);

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
      "YawControlActionServer: rob=(%.3f,%.3f,%.3f deg) "
      "traveled=%.3f remaining=%.3f vx=%.3f omega=%.3f e_theta=%.3f deg phase=%u",
      rob_x, rob_y, rob_yaw * 180.0 / M_PI,
      traveled, remaining, vx, omega,
      e_theta * 180.0 / M_PI,
      static_cast<unsigned>(profile_out.phase));

    // ── n. Publish feedback ───────────────────────────────────────────────
    {
      const double e_lateral = -(dx_r * uy - dy_r * ux);
      auto feedback = std::make_shared<YawControl::Feedback>();
      feedback->current_distance      = traveled;
      feedback->current_lateral_error = e_lateral;
      feedback->current_heading_error = e_theta * 180.0 / M_PI;
      feedback->current_vx            = vx;
      feedback->current_vy            = 0.0;
      feedback->current_omega         = omega;
      feedback->phase                 = static_cast<uint8_t>(profile_out.phase);
      goal_handle->publish_feedback(feedback);
    }

    rate.sleep();
  }

  // rclcpp::ok() went false — shutdown
  publishCmdVel(0.0, 0.0);
}

}  // namespace amr_motion_control_2wd
