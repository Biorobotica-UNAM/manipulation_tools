#pragma once

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "manipulation_planner_interfaces/srv/solve_right_arm_ik.hpp"

class Manipulation
{
public:
  explicit Manipulation(const rclcpp::Node::SharedPtr& node);

  bool SolveRightArmIK(
    const geometry_msgs::msg::PoseStamped& target_pose,
    std::vector<double>& joint_positions,
    std::string& message);

private:
  bool waitForService(
    const rclcpp::ClientBase::SharedPtr& client,
    const std::chrono::milliseconds& timeout) const;

  rclcpp::Node::SharedPtr node_;

  rclcpp::Client<
    manipulation_planner_interfaces::srv::SolveRightArmIK>::SharedPtr
    ik_client_;
};

