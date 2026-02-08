#include "manipulation_tools/Manipulation.hpp"

using namespace std::chrono_literals;

Manipulation::Manipulation(const rclcpp::Node::SharedPtr& node)
	: node_(node)
{
	ik_client_ = node_->create_client<manipulation_planner_interfaces::srv::SolveRightArmIK>("/manipulation/manipulation_planner/solve_right_arm_ik");
}

// ---------------------------------------------------------
// Wait for service
// ---------------------------------------------------------

bool Manipulation::waitForService(
	const rclcpp::ClientBase::SharedPtr& client,
	const std::chrono::milliseconds& timeout) const
{
	return client->wait_for_service(timeout);
}

// ---------------------------------------------------------
// Right arm inverse kinematic solver
// ---------------------------------------------------------

bool Manipulation::SolveRightArmIK(const geometry_msgs::msg::PoseStamped& target_pose, std::vector<double>& joint_positions, std::string& message)
{
	if (!waitForService(talker_client_, 500ms))
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "Talk service unavailable (/talk)");
        return false;
    }
	auto request = std::make_shared<manipulation_planner_interfaces::srv::SolveRightArmIK::Request>();
	request->target_pose = target_pose;

	std::mutex mtx;
    std::condition_variable cv;
    bool done = false;
    bool success = false;	
	
	auto callback =[&](rclcpp::Client<manipulation_planner_interfaces::srv::SolveRightArmIK>::SharedFuture future)
        {
            auto response = future.get();
            joint_positions = response->joint_positions;
            message = response->message;
            success = response->success;

            done = true;
            cv.notify_one();
        };

	ik_client_->async_send_request(request, callback);

	std::unique_lock<std::mutex> lock(mtx);

	if (!cv.wait_for(lock, 10s, [&] { return done; }))
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "IK service timed out");
        return false;
    }

	return success;
}
