#include "manipulation_tools/Manipulation.hpp"

Manipulation::Manipulation(rclcpp::Node::SharedPtr node)
	: node(node) {
	arm_movement =  node->create_client<manipulation_planner::srv::ArmMovementPlan>("/plan_arm_movement", rclcpp::QoS(10));

	// Wait up to 10 seconds for service
    if (!client_->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node->get_logger(), "Plan Arm Movement not available");
        return;
    }

    RCLCPP_INFO(node->get_logger(), "Plan Arm Movement service is available!");
}

auto Manipulation::HandleMoveArmResponse(rclcpp::Client<manipulation_planner::srv::ArmMovementPlan>::SharedFuture future) {
	auto response = future.get();
	RCLCPP_INFO(node->get_logger(), response->message.c_str());
}

void Manipulation::MoveArmTo(std::string pose_name) {
	auto request = std::make_shared<manipulation_planner::srv::ArmMovementPlan::Request>();
	request->pose_name = pose_name;

	auto callback = std::bind(&Manipulation::HandleMoveArmResponse,
		this,
		std::placeholders::_1);

	arm_movement->async_send_request(request, callback);
}

void Manipulation::MoveArmTo(double x, double y, double z, 
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
}