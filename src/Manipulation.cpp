#include "manipulation_tools/Manipulation.hpp"
using namespace std::chrono_literals;

Manipulation::Manipulation(const rclcpp::Node::SharedPtr& node)
: node_(node)
{
    plan_trajectory_client_ = node_->create_client<
        manipulation_planner_interfaces::srv::PlanTrajectory>(
        "/manipulation/manipulation_planner/plan_trajectory");

    follow_trajectory_client_ = node_->create_client<
        manipulation_controller_interfaces::srv::FollowTrajectory>(
        "/manipulation/manipulation_controller/follow_trajectory");
}

// ---------------------------------------------------------
// Plan Trajectory
// ---------------------------------------------------------
bool Manipulation::PlanTrajectory(
    const geometry_msgs::msg::PoseStamped& target_pose,
    const std::string& planning_group,
    std::vector<std::string>& joint_names,
    std::vector<double>& waypoints,
    std::string& message)
{
    if (!plan_trajectory_client_->wait_for_service(1000ms)) {
        RCLCPP_ERROR(node_->get_logger(),
            "Plan trajectory service unavailable (/plan_trajectory)");
        return false;
    }

    auto request = std::make_shared<
        manipulation_planner_interfaces::srv::PlanTrajectory::Request>();
    request->target_pose    = target_pose;
    request->planning_group = planning_group;
    request->planning_time  = 5.0;

    std::mutex mtx;
    std::condition_variable cv;
    bool done    = false;
    bool success = false;

    auto callback = [&](rclcpp::Client<
        manipulation_planner_interfaces::srv::PlanTrajectory>::SharedFuture future)
    {
        auto response = future.get();

        success     = response->success;
        message     = response->message;
        joint_names = response->joint_names;
        waypoints   = response->waypoints;

        done = true;
        cv.notify_one();
    };

    plan_trajectory_client_->async_send_request(request, callback);

    std::unique_lock<std::mutex> lock(mtx);
    if (!cv.wait_for(lock, 30s, [&]{ return done; })) {
        RCLCPP_ERROR(node_->get_logger(), "Plan trajectory service timed out");
        return false;
    }

    if (success)
        RCLCPP_INFO(node_->get_logger(), "Trajectory OK: %s", message.c_str());
    else
        RCLCPP_ERROR(node_->get_logger(), "Trajectory ERROR: %s", message.c_str());

    return success;
}

// ---------------------------------------------------------
// Follow Trajectory
// ---------------------------------------------------------
bool Manipulation::FollowTrajectory(
    const std::string& planning_group,
    const std::vector<std::string>& joint_names,
    const std::vector<double>& waypoints,
    std::string& message)
{
    if (!follow_trajectory_client_->wait_for_service(1000ms)) {
        RCLCPP_ERROR(node_->get_logger(),
            "Follow trajectory service unavailable (/follow_trajectory)");
        return false;
    }

    auto request = std::make_shared<
        manipulation_controller_interfaces::srv::FollowTrajectory::Request>();

    request->planning_group = planning_group;
    request->joint_names    = joint_names;
    request->waypoints      = waypoints;

    std::mutex mtx;
    std::condition_variable cv;
    bool done    = false;
    bool success = false;

    auto callback = [&](rclcpp::Client<
        manipulation_controller_interfaces::srv::FollowTrajectory>::SharedFuture future)
    {
        auto response = future.get();

        success = response->success;
        message = response->message;

        done = true;
        cv.notify_one();
    };

    follow_trajectory_client_->async_send_request(request, callback);

    std::unique_lock<std::mutex> lock(mtx);
    if (!cv.wait_for(lock, 60s, [&]{ return done; })) {
        RCLCPP_ERROR(node_->get_logger(), "Follow trajectory service timed out");
        return false;
    }

    if (success)
        RCLCPP_INFO(node_->get_logger(), "Execution OK: %s", message.c_str());
    else
        RCLCPP_ERROR(node_->get_logger(), "Execution ERROR: %s", message.c_str());

    return success;
}
