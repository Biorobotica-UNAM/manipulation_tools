#include "manipulation_tools/Manipulation.hpp"

Manipulation::Manipulation(rclcpp::Node::SharedPtr node)
	: node(node) {
	arm_movement =  node->create_publisher<std_msgs::msg::Float64MultiArray>("/hardware/arms/goal_pose", rclcpp::QoS(10));
}

/*auto Manipulation::HandleMoveArmResponse(rclcpp::Client<manipulation_planner::srv::ArmMovementPlan>::SharedFuture future) {
	auto response = future.get();
	RCLCPP_INFO(node->get_logger(), response->message.c_str());
}*/

void Manipulation::MoveArmTo(std::string pose_name) {
	auto msg = std_msgs::msg::Float64MultiArray();

	if(pose_name = "neutral") {
		msg.data = {0.0, 0.0, 0.0, 0.0};
	} else if(pose_name = "drop") {
		msg.data = {0.384, -0.2094, 0.2443, 0.0};
	} else if(pose_name = "take") {
		msg.data = {0.4712, 0.0, 0.7854, 0.0};
	} else {
		RCLCPP_INFO(node->get_logger(), "POSE NOT DEFINED");
	}

	arm_movement->publish(msg);
}

/*void Manipulation::MoveArmTo(double x, double y, double z, 
		double roll, double pitch, double yaw) {
	auto request = std::make_shared<manipulation_planner::srv::ArmMovementPlan::Request>();
	request->x = x;
	request->y = y;
	request->z = z;
	request->roll = roll;
	request->pitch = pitch;
	request->yaw = yaw;

	auto callback = std::bind(&Manipulation::HandleMoveArmResponse,
		this,
		std::placeholders::_1);

	arm_movement->async_send_request(request, callback);
}*/