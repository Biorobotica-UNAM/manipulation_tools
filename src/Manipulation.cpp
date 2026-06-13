#include "manipulation_tools/Manipulation.hpp"

Manipulation::Manipulation(const rclcpp::Node::SharedPtr &node)
    : node_(node)
{
  cb_group_clients_ =
      node_->create_callback_group(
          rclcpp::CallbackGroupType::Reentrant);

  const rclcpp::QoS qos = rclcpp::ServicesQoS();

  plan_trajectory_client_ =
      node_->create_client<
          manipulation_planner_interfaces::srv::PlanTrajectory>(
          "/manipulation/manipulation_planner/plan_trajectory",
          qos,
          cb_group_clients_);

  follow_trajectory_client_ =
      node_->create_client<
          manipulation_controller_interfaces::srv::FollowTrajectory>(
          "/manipulation/manipulation_controller/follow_trajectory",
          qos,
          cb_group_clients_);
}

bool Manipulation::PlanTrajectory(
    const geometry_msgs::msg::PoseStamped &target_pose,
    const std::string &planning_group,
	const std::string &tip_link,
    std::vector<std::string> &joint_names,
    std::vector<double> &waypoints,
    std::string &message,
    bool enable_target_exclusion,
    const geometry_msgs::msg::PoseStamped &target_exclusion_center,
    double target_exclusion_radius_xy,
    double target_exclusion_height_z,
    std::chrono::seconds timeout)
{
  auto request = std::make_shared<
      manipulation_planner_interfaces::srv::PlanTrajectory::Request>();

  request->target_pose = target_pose;
  request->planning_group = planning_group;
  request->tip_link = tip_link; 
  request->planning_time = 15.0;
  request->enable_target_exclusion = enable_target_exclusion;
  request->target_exclusion_center = target_exclusion_center;
  request->target_exclusion_radius_xy = target_exclusion_radius_xy;
  request->target_exclusion_height_z = target_exclusion_height_z;

  bool success = false;

  bool completed =
      call_service<
          manipulation_planner_interfaces::srv::PlanTrajectory>(
          plan_trajectory_client_,
          request,
          [&](auto response)
          {
            success = response->success;
            message = response->message;
            joint_names = response->joint_names;
            waypoints = response->waypoints;

            RCLCPP_INFO(
                node_->get_logger(),
                "Trajectory %s: %s",
                success ? "OK" : "ERROR",
                message.c_str());
          },
          timeout,
          "PlanTrajectory");

  return completed && success;
}

bool Manipulation::FollowTrajectory(
    const std::string &planning_group,
    const std::vector<std::string> &joint_names,
    const std::vector<double> &waypoints,
    std::string &message,
    std::chrono::seconds timeout)
{
  auto request = std::make_shared<
      manipulation_controller_interfaces::srv::FollowTrajectory::Request>();

  request->planning_group = planning_group;
  request->joint_names = joint_names;
  request->waypoints = waypoints;

  bool success = false;

  bool completed =
      call_service<
          manipulation_controller_interfaces::srv::FollowTrajectory>(
          follow_trajectory_client_,
          request,
          [&](auto response)
          {
            success = response->success;
            message = response->message;

            RCLCPP_INFO(
                node_->get_logger(),
                "Execution %s: %s",
                success ? "OK" : "ERROR",
                message.c_str());
          },
          timeout,
          "FollowTrajectory");

  return completed && success;
}
