#pragma once

/**
 * @brief Facade over ROS2 manipulation services.
 *
 * Responsibility:
 *   Provide a synchronous and intention-revealing API for
 *   trajectory planning and execution without exposing ROS2
 *   async service mechanics to callers.
 *
 * Design patterns applied:
 *   Facade          — hides ROS2 service clients.
 *   Template Method — call_service<T>() centralizes the
 *                     async → wait → return flow.
 *
 * Principles applied:
 *   SRP — one class responsible for manipulation communication.
 *   DRY — timeout/wait logic implemented once.
 *   DIP — callers depend on Manipulation, not ROS2 clients.
 *
 * BioRobotics Laboratory, Facultad de Ingenieria, UNAM
 */

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "manipulation_controller_interfaces/srv/follow_trajectory.hpp"
#include "manipulation_planner_interfaces/srv/plan_trajectory.hpp"

using namespace std::chrono_literals;

class Manipulation {
public:
  explicit Manipulation(const rclcpp::Node::SharedPtr &node);

  /**
   * Plan a trajectory towards a target pose.
   *
   * @param target_pose     Desired end-effector pose.
   * @param planning_group  MoveIt planning group.
   * @param tip_link        MoveIt kinematic chain tip.
   * @param joint_names     Output joint names.
   * @param waypoints       Output waypoint vector.
   * @param message         Planner status message.
   * @param enable_target_exclusion  Ignore the target object in MoveIt's cloud.
   * @param target_exclusion_center  Object center to exclude from the cloud.
   * @param target_exclusion_radius_xy  Horizontal exclusion radius in meters.
   * @param target_exclusion_height_z   Vertical exclusion height in meters.
   * @param timeout         Maximum wait time.
   *
   * @return true if planning succeeded.
   */
  bool PlanTrajectory(
      const geometry_msgs::msg::PoseStamped &target_pose,
      const std::string &planning_group,
      const std::string &tip_link,
      std::vector<std::string> &joint_names,
      std::vector<double> &waypoints,
      std::string &message,
      bool enable_target_exclusion = false,
      const geometry_msgs::msg::PoseStamped &target_exclusion_center =
          geometry_msgs::msg::PoseStamped(),
      double target_exclusion_radius_xy = 0.14,
      double target_exclusion_height_z = 0.24,
      std::chrono::seconds timeout = 30s);

  /**
   * Execute a planned trajectory.
   *
   * @param planning_group  MoveIt planning group.
   * @param joint_names     Joint names sequence.
   * @param waypoints       Flattened waypoint vector.
   * @param message         Controller status message.
   * @param timeout         Maximum wait time.
   *
   * @return true if execution succeeded.
   */
  bool FollowTrajectory(
      const std::string &planning_group,
      const std::vector<std::string> &joint_names,
      const std::vector<double> &waypoints,
      std::string &message,
      std::chrono::seconds timeout = 60s);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr cb_group_clients_;

  rclcpp::Client<
      manipulation_planner_interfaces::srv::PlanTrajectory>::SharedPtr
      plan_trajectory_client_;

  rclcpp::Client<
      manipulation_controller_interfaces::srv::FollowTrajectory>::SharedPtr
      follow_trajectory_client_;

  /**
   * Template Method:
   * invariant async → wait → return flow.
   */
  template <typename ServiceT, typename CallbackT>
  bool call_service(
      typename rclcpp::Client<ServiceT>::SharedPtr &client,
      typename ServiceT::Request::SharedPtr request,
      CallbackT on_response,
      std::chrono::seconds timeout,
      const std::string &label);
};

// ============================================================
// Template implementation
// ============================================================

template <typename ServiceT, typename CallbackT>
bool Manipulation::call_service(
    typename rclcpp::Client<ServiceT>::SharedPtr &client,
    typename ServiceT::Request::SharedPtr request,
    CallbackT on_response,
    std::chrono::seconds timeout,
    const std::string &label)
{
  if (!client->wait_for_service(1s)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "%s service not available.",
                 label.c_str());
    return false;
  }

  auto future = client->async_send_request(request);

  if (future.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Timeout calling %s service after %ld s.",
                 label.c_str(),
                 timeout.count());
    return false;
  }

  on_response(future.get());

  return true;
}
