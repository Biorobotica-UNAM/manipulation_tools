#include "manipulation_tools/Manipulation.hpp"

using namespace std::chrono_literals;

Manipulation::Manipulation(const rclcpp::Node::SharedPtr& node)
: node_(node)
{
  ik_client_ = node_->create_client<
    manipulation_planner_interfaces::srv::SolveRightArmIK>(
      "/manipulation_planner/solve_right_arm_ik");
}

bool Manipulation::SolveRightArmIK(
  const geometry_msgs::msg::PoseStamped& target_pose,
  std::vector<double>& joint_positions,
  std::string& message)
{
  if (!waitForService(ik_client_, 1000ms))
  {
    RCLCPP_ERROR(node_->get_logger(),
      "IK service unavailable (/manipulation_planner/solve_right_arm_ik)");
    return false;
  }

  auto request = std::make_shared<
    manipulation_planner_interfaces::srv::SolveRightArmIK::Request>();

  request->target_pose = target_pose;

  auto future = ik_client_->async_send_request(request);

  auto state = rclcpp::spin_until_future_complete(
    node_, future, 5s);

  if (state != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(),
      "No response from IK service");
    return false;
  }

  auto response = future.get();

  joint_positions = response->joint_positions;
  message = response->message;

  return response->success;
}

bool Manipulation::waitForService(
  const rclcpp::ClientBase::SharedPtr& client,
  const std::chrono::milliseconds& timeout) const
{
  return client->wait_for_service(timeout);
}

