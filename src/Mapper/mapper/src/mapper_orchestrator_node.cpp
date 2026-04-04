#include "mapper/mapper_orchestrator_node.hpp"
#include <chrono>

namespace mapper {

MapperOrchestratorNode::MapperOrchestratorNode(
    const rclcpp::NodeOptions & options)
: Node("mapper_orchestrator_node", options)
{
    using namespace std::placeholders;

    status_pub_ = create_publisher<mapper_interfaces::msg::MapperStatus>(
        "mapper/status", rclcpp::QoS(10).reliable());

    auto cb_group = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    cmd_service_ = create_service<mapper_interfaces::srv::MapperCommand>(
        "mapper/command",
        std::bind(&MapperOrchestratorNode::handle_command, this, _1, _2),
        rmw_qos_profile_services_default, cb_group);

    wall_align_client_ = rclcpp_action::create_client<
        mapper_interfaces::action::WallAlign>(this, "wall_align");
    map_check_client_  = rclcpp_action::create_client<
        mapper_interfaces::action::MapAlignmentCheck>(this, "map_alignment_check");
    explore_client_    = rclcpp_action::create_client<
        mapper_interfaces::action::ExploreUnknown>(this, "explore_unknown");
    slam_ctrl_client_  = create_client<mapper_interfaces::srv::SlamControl>(
        "slam_manager_2d/slam_control");

    status_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MapperOrchestratorNode::publish_status, this));
}

void MapperOrchestratorNode::transition_to(MapperState new_state) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    RCLCPP_INFO(get_logger(), "State: %d -> %d",
        static_cast<int>(state_.load()), static_cast<int>(new_state));
    state_.store(new_state);
}

void MapperOrchestratorNode::log(const std::string & msg) {
    RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    log_message_ = msg;
}

void MapperOrchestratorNode::publish_status() {
    mapper_interfaces::msg::MapperStatus status;
    status.state                   = static_cast<uint8_t>(state_.load());
    status.previous_state          = static_cast<uint8_t>(previous_state_);
    status.coverage_percent        = coverage_percent_;
    status.current_heading_error_deg = heading_error_deg_;
    status.log_message             = log_message_;
    status_pub_->publish(status);
}

void MapperOrchestratorNode::handle_command(
    const mapper_interfaces::srv::MapperCommand::Request::SharedPtr req,
    mapper_interfaces::srv::MapperCommand::Response::SharedPtr res)
{
    using Cmd = mapper_interfaces::srv::MapperCommand::Request;
    auto state = state_.load();

    switch (req->command) {
    case Cmd::CMD_START_MAPPING:
        if (state == MapperState::IDLE) {
            slam_mode_  = req->slam_mode;
            drive_mode_ = req->drive_mode;
            align_retry_count_ = 0;
            transition_to(MapperState::ALIGNING);
            std::thread([this]{ run_aligning(); }).detach();
            res->success = true;
            res->message = "Mapping started";
        } else {
            res->success = false;
            res->message = "Not in IDLE state";
        }
        break;

    case Cmd::CMD_PAUSE:
        if (state == MapperState::MAPPING_MANUAL ||
            state == MapperState::MAPPING_AUTO ||
            state == MapperState::EXPLORING_UNKNOWN) {
            previous_state_ = state;
            transition_to(MapperState::PAUSED);
            res->success = true;
            res->message = "Paused";
        } else {
            res->success = false;
            res->message = "Cannot pause in current state";
        }
        break;

    case Cmd::CMD_RESUME:
        if (state == MapperState::PAUSED) {
            auto prev = previous_state_;
            transition_to(prev);
            if (prev == MapperState::MAPPING_MANUAL)
                std::thread([this]{ run_mapping_manual(); }).detach();
            else if (prev == MapperState::EXPLORING_UNKNOWN)
                std::thread([this]{ run_exploring(); }).detach();
            res->success = true;
            res->message = "Resumed";
        } else {
            res->success = false;
            res->message = "Not paused";
        }
        break;

    case Cmd::CMD_STOP:
        transition_to(MapperState::IDLE);
        if (wall_align_client_) wall_align_client_->async_cancel_all_goals();
        if (map_check_client_)  map_check_client_->async_cancel_all_goals();
        if (explore_client_)    explore_client_->async_cancel_all_goals();
        res->success = true;
        res->message = "Stopped";
        break;

    case Cmd::CMD_EXPLORE:
        if (state == MapperState::MAPPING_MANUAL) {
            transition_to(MapperState::EXPLORING_UNKNOWN);
            std::thread([this]{ run_exploring(); }).detach();
            res->success = true;
            res->message = "Exploration started";
        } else {
            res->success = false;
            res->message = "Must be in MAPPING_MANUAL state";
        }
        break;

    default:
        res->success = false;
        res->message = "Unknown command";
    }
}

void MapperOrchestratorNode::run_aligning() {
    if (state_.load() == MapperState::IDLE) return;

    // Poll for action server with early exit on IDLE/stop
    bool server_ready = false;
    for (int i = 0; i < 50 && rclcpp::ok(); ++i) {
        if (state_.load() == MapperState::IDLE) return;
        if (wall_align_client_->action_server_is_ready()) {
            server_ready = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!server_ready) {
        if (state_.load() == MapperState::IDLE) return;
        log("WallAligner not available");
        transition_to(MapperState::ERROR);
        return;
    }

    mapper_interfaces::action::WallAlign::Goal goal{};
    goal.tolerance_deg      = 0.2;
    goal.use_imu_correction = false;

    std::atomic<bool> done{false};
    std::atomic<int8_t> result_status{-1};

    auto opts = rclcpp_action::Client<
        mapper_interfaces::action::WallAlign>::SendGoalOptions{};
    opts.result_callback = [&done, &result_status](const auto & r) {
        result_status.store(r.result->status);
        done.store(true);
    };

    wall_align_client_->async_send_goal(goal, opts);
    while (!done.load() && rclcpp::ok() &&
           state_.load() != MapperState::IDLE) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (state_.load() == MapperState::IDLE) return;

    if (result_status.load() == 0) {
        align_retry_count_ = 0;
        transition_to(MapperState::STARTING_SLAM);
        run_starting_slam();
    } else {
        ++align_retry_count_;
        if (align_retry_count_ >= max_align_retries_) {
            log("Alignment failed after max retries");
            transition_to(MapperState::ERROR);
        } else {
            run_aligning();
        }
    }
}

void MapperOrchestratorNode::run_starting_slam() {
    if (state_.load() == MapperState::IDLE) return;

    bool svc_ready = false;
    for (int i = 0; i < 50 && rclcpp::ok(); ++i) {
        if (state_.load() == MapperState::IDLE) return;
        if (slam_ctrl_client_->service_is_ready()) { svc_ready = true; break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!svc_ready) {
        if (state_.load() == MapperState::IDLE) return;
        log("SlamControl service not available");
        transition_to(MapperState::ERROR);
        return;
    }

    using SlamCtrl = mapper_interfaces::srv::SlamControl;
    auto req = std::make_shared<SlamCtrl::Request>();
    req->command = (slam_mode_ == 0) ?
        SlamCtrl::Request::CMD_START_2D :
        SlamCtrl::Request::CMD_START_3D;

    std::atomic<bool> done{false};
    std::atomic<bool> success{false};
    slam_ctrl_client_->async_send_request(req,
        [&done, &success](rclcpp::Client<SlamCtrl>::SharedFuture fut) {
            success.store(fut.get()->success);
            done.store(true);
        });
    while (!done.load() && rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!success.load()) {
        log("SLAM start failed");
        transition_to(MapperState::ERROR);
        return;
    }

    std::this_thread::sleep_for(
        std::chrono::duration<double>(map_stabilize_wait_sec_));

    transition_to(MapperState::VERIFYING_MAP);
    run_verifying_map();
}

void MapperOrchestratorNode::run_verifying_map() {
    if (state_.load() == MapperState::IDLE) return;

    bool server_ready = false;
    for (int i = 0; i < 50 && rclcpp::ok(); ++i) {
        if (state_.load() == MapperState::IDLE) return;
        if (map_check_client_->action_server_is_ready()) { server_ready = true; break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!server_ready) {
        if (state_.load() == MapperState::IDLE) return;
        log("MapAlignmentChecker not available -- skipping");
        transition_to(MapperState::MAPPING_MANUAL);
        return;
    }

    mapper_interfaces::action::MapAlignmentCheck::Goal goal{};
    goal.tolerance_deg = 0.5;

    std::atomic<bool> done{false};
    bool is_aligned = true;

    auto opts = rclcpp_action::Client<
        mapper_interfaces::action::MapAlignmentCheck>::SendGoalOptions{};
    opts.result_callback = [&done, &is_aligned](const auto & r) {
        is_aligned = r.result->is_aligned;
        done.store(true);
    };

    map_check_client_->async_send_goal(goal, opts);
    while (!done.load() && rclcpp::ok() &&
           state_.load() != MapperState::IDLE) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (state_.load() == MapperState::IDLE) return;

    if (is_aligned) {
        log("Map alignment verified");
        transition_to(MapperState::MAPPING_MANUAL);
    } else {
        log("Map misaligned -- re-aligning");
        align_retry_count_ = 0;
        transition_to(MapperState::ALIGNING);
        run_aligning();
    }
}

void MapperOrchestratorNode::run_mapping_manual() {
    log("Manual mapping mode -- waiting for commands");
}

void MapperOrchestratorNode::run_exploring() {
    if (state_.load() == MapperState::IDLE) return;

    bool server_ready = false;
    for (int i = 0; i < 50 && rclcpp::ok(); ++i) {
        if (state_.load() == MapperState::IDLE) return;
        if (explore_client_->action_server_is_ready()) { server_ready = true; break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!server_ready) {
        if (state_.load() == MapperState::IDLE) return;
        log("ExplorationPlanner not available");
        transition_to(MapperState::ERROR);
        return;
    }

    mapper_interfaces::action::ExploreUnknown::Goal goal{};
    goal.mode = drive_mode_;
    goal.min_coverage_to_stop = min_coverage_to_stop_;

    std::atomic<bool> done{false};
    std::atomic<int8_t> result_status{-1};

    auto opts = rclcpp_action::Client<
        mapper_interfaces::action::ExploreUnknown>::SendGoalOptions{};
    opts.feedback_callback = [this](auto, const auto & fb) {
        coverage_percent_ = fb->coverage_percent;
    };
    opts.result_callback = [&done, &result_status](const auto & r) {
        result_status.store(r.result->status);
        done.store(true);
    };

    explore_client_->async_send_goal(goal, opts);
    while (!done.load() && rclcpp::ok() &&
           state_.load() != MapperState::IDLE &&
           state_.load() != MapperState::PAUSED) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (state_.load() == MapperState::IDLE) return;
    if (state_.load() == MapperState::PAUSED) return;

    if (result_status.load() == 0) {
        transition_to(MapperState::LOOP_CLOSING);
        run_loop_closing();
    } else {
        log("Exploration failed");
        transition_to(MapperState::MAPPING_MANUAL);
    }
}

void MapperOrchestratorNode::run_loop_closing() {
    log("Waiting for loop closure...");
    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= static_cast<long>(loop_closure_timeout_sec_)) {
            log("Loop closure timeout -- completing");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    transition_to(MapperState::COMPLETED);
    log("Mapping completed");
}

}  // namespace mapper
