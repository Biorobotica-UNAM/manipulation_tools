#pragma once

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "manipulation_planner_interfaces/srv/plan_trajectory.hpp"

class Manipulation
{
public:
    explicit Manipulation(const rclcpp::Node::SharedPtr& node);

    bool PlanTrajectory(
        const geometry_msgs::msg::PoseStamped& target_pose,
        const std::string& planning_group,
        std::vector<std::string>& joint_names,
        std::vector<double>& waypoints,
        std::vector<double>& timestamps,
        std::string& message);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<manipulation_planner_interfaces::srv::PlanTrajectory>::SharedPtr
        plan_trajectory_client_;
};
