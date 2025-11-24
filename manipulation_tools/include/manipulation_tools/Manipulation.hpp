#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "manipulation_planner/srv/arm_movement_plan.hpp"

class Manipulation {
private:
	// Constructor
	Manipulation(rclcpp::Node::SharedPtr node);

	HandleMoveArmResponse(rclcpp::Client<manipulation_planner::srv::ArmMovementPlan>::SharedFuture future);

	// Service clients
	rclcpp::Client<manipulation_planner::srv::ArmMovementPlan>::SharedPtr arm_movement;

	// Node
	rclcpp::Node::SharedPtr node;

public:
	// Overloaded method MoveArmTo to move the arm either with
	// a predefined pose or the pose values. Both call the same
	// service.
	MoveArmTo(std::string pose_name);

	MoveArmTo(double x, double y, double z, 
		double roll, double pitch, double yaw);

	// OpenGripper();

	// CloseGripper(); 
}