#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "manipulation_planner/srv/arm_movement_plan.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class Manipulation
{
public:
	// Constructor
	Manipulation(rclcpp::Node::SharedPtr node);

	// Overloaded method MoveArmTo to move the arm either with
	// a predefined pose or the pose values. Both call the same
	// service.
	void MoveArmTo(std::string pose_name);

	// void MoveArmTo(double x, double y, double z, 
	//	double roll, double pitch, double yaw);

private:
	// auto HandleMoveArmResponse(rclcpp::Client<manipulation_planner::srv::ArmMovementPlan>::SharedFuture future);

	// Service clients
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_movement;

	// Node
	rclcpp::Node::SharedPtr node;
};