/**
 * @file navigation_client.cpp
 * @author Grayson Arendt
 * @date 10/2/2024
 */

#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "lunabot_system/action/localization.hpp"

/**
 * @class NavigationClient
 * @brief Sends navigation goals, handles received localization data, and controls robot movement based on odometry feedback.
 */
class NavigationClient : public rclcpp::Node
{
public:
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;
  using Localization = lunabot_system::action::Localization;
  using GoalHandleLocalization = rclcpp_action::ClientGoalHandle<Localization>;

  /**
   * @brief Constructor for the NavigationClient class.
   */
  NavigationClient()
      : Node("navigator_client"), goal_reached_(false), localization_done_(false)
  {
    nav_through_poses_client_ = rclcpp_action::create_client<NavigateThroughPoses>(this, "navigate_through_poses");
    localization_client_ = rclcpp_action::create_client<Localization>(this, "localization_action");
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&NavigationClient::odom_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&NavigationClient::localize_and_send_goal, this));
  }

private:
  /**
   * @brief Calls the localization action and starts the navigation to the first goal.
   */
  void localize_and_send_goal()
  {
    if (!localization_done_)
    {
      if (!localization_client_->wait_for_action_server(std::chrono::seconds(10)))
      {
        RCLCPP_ERROR(this->get_logger(), "LOCALIZATION ACTION SERVER NOT AVAILABLE.");
        rclcpp::shutdown();
        return;
      }

      auto goal_msg = Localization::Goal();
      auto send_goal_options = rclcpp_action::Client<Localization>::SendGoalOptions();
      send_goal_options.result_callback = std::bind(&NavigationClient::handle_localization_result, this, std::placeholders::_1);
      localization_client_->async_send_goal(goal_msg, send_goal_options);
    }

    if (!goal_reached_)
    {
      send_navigation_through_poses();
    }
  }

  /**
   * @brief Handles the result from the localization action.
   * @param result The result from the localization action.
   */
  void handle_localization_result(const GoalHandleLocalization::WrappedResult &result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      initial_x_ = result.result->x;
      initial_y_ = result.result->y;
      localization_done_ = true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "LOCALIZATION FAILED.");
      rclcpp::shutdown();
    }
  }

  /**
   * @brief Sends the navigation through poses.
   */
  void send_navigation_through_poses()
  {
    if (!this->nav_through_poses_client_->wait_for_action_server())
    {
      RCLCPP_ERROR(this->get_logger(), "NAV2 ACTION SERVER NOT AVAILABLE.");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = NavigateThroughPoses::Goal();
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;

    geometry_msgs::msg::PoseStamped pose1;
    pose1.header.stamp = this->now();
    pose1.header.frame_id = "map";
    pose1.pose.position.x = initial_x_ + 3.3;
    pose1.pose.position.y = initial_y_ + 2.2;
    pose1.pose.orientation.z = -0.707;
    pose1.pose.orientation.w = 0.707;
    waypoints.push_back(pose1);

    geometry_msgs::msg::PoseStamped pose2;
    pose2.header.stamp = this->now();
    pose2.header.frame_id = "map";
    pose2.pose.position.x = 4.03;
    pose2.pose.position.y = 0.6;
    pose2.pose.orientation.z = 0.707;
    pose2.pose.orientation.w = 0.707;
    waypoints.push_back(pose2);

    goal_msg.poses = waypoints;

    auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&NavigationClient::goal_result_callback, this, std::placeholders::_1);

    this->nav_through_poses_client_->async_send_goal(goal_msg, send_goal_options);
  }

  /**
   * @brief Callback for the result of the navigation through poses.
   * @param result The result of the goal execution.
   */
  void goal_result_callback(const GoalHandleNavigate::WrappedResult &result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      goal_reached_ = true;
      RCLCPP_INFO(this->get_logger(), "\033[1;32mCONSTRUCTION ZONE GOAL SUCCESS!\033[0m");
    }
  }

  /**
   * @brief Odometry callback.
   * @param msg Odometry message.
   */
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Handle odometry updates if needed for additional logic
  }

  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav_through_poses_client_;
  rclcpp_action::Client<Localization>::SharedPtr localization_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool goal_reached_, localization_done_;
  double initial_x_, initial_y_;
};

/**
 * @brief Main function.
 * Initializes and runs the NavigationClient node.
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationClient>());
  rclcpp::shutdown();
  return 0;
}
