#pragma once

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "manipulation_planner_interfaces/srv/plan_trajectory.hpp"
#include "manipulation_controller_interfaces/srv/follow_trajectory.hpp"

class Manipulation
{
public:
    explicit Manipulation(const rclcpp::Node::SharedPtr& node);

    bool PlanTrajectory(
        const geometry_msgs::msg::PoseStamped& target_pose,
        const std::string& planning_group,
        std::vector<std::string>& joint_names,
        std::vector<double>& waypoints,
        std::string& message);

    bool FollowTrajectory(
        const std::string& planning_group,
        const std::vector<std::string>& joint_names,
        const std::vector<double>& waypoints,
        std::string& message);

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Client<
        manipulation_planner_interfaces::srv::PlanTrajectory>::SharedPtr
        plan_trajectory_client_;

    rclcpp::Client<
        manipulation_controller_interfaces::srv::FollowTrajectory>::SharedPtr
        follow_trajectory_client_;
};
