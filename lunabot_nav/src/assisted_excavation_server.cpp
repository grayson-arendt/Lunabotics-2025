/**
 * @file assisted_excavation_server.cpp
 * @author Grayson Arendt
 * @date 02/22/2026
 */

#include "lunabot_logger/logger.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "geometry_msgs/msg/twist.hpp"
#include "lunabot_msgs/action/excavation.hpp"
#include "lunabot_msgs/msg/motor_commands.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <std_msgs/msg/float64.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

static constexpr double excavation_pos = 1.59;
static constexpr double travel_pos = 1.0;
static constexpr int forward_seconds = 5;

/**
 * @class AssistedExcavationServer
 * @brief Assisted excavation server for teleop that controls bucket actuators and manages
 * excavation sequence.
 */
class AssistedExcavationServer : public rclcpp::Node
{
public:
  using Excavation = lunabot_msgs::action::Excavation;
  using GoalHandleExcavation = rclcpp_action::ServerGoalHandle<Excavation>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  /**
   * @brief Constructor for the AssistedExcavationServer class.
   */
  AssistedExcavationServer() : Node("assisted_excavation_server")
  {
    encoder_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    action_server_ = rclcpp_action::create_server<Excavation>(
      this, "assisted_excavation_action",
      [this](const auto &, const auto &) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const auto &) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](const auto goal_handle) {
        std::thread{[this, goal_handle]() { execute(goal_handle); }}.detach();
      });

    navigation_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = encoder_callback_group_;

    encoder_position_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "bucket_angle", 10,
      std::bind(&AssistedExcavationServer::encoder_position_callback, this, std::placeholders::_1),
      sub_options);

    excavation_angle_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "/excavation_angle", 10,
      std::bind(&AssistedExcavationServer::excavation_angle_callback, this, std::placeholders::_1));

    motor_cmd_publisher_ =
      this->create_publisher<lunabot_msgs::msg::MotorCommands>("/motor_commands", 10);

    LOGGER_SUCCESS(this->get_logger(), "Assisted excavation server initialized");
  }

private:
  /**
   * @brief Lowers bucket for excavation.
   * @return true if successful, false if canceled
   */
  bool lower_bucket(const std::shared_ptr<GoalHandleExcavation> goal_handle)
  {
    LOGGER_ACTION(this->get_logger(), "Lowering bucket...");

    double target_position = excavation_angle_;

    auto motor_cmd = lunabot_msgs::msg::MotorCommands();
    motor_cmd.left_wheel = 0.0;
    motor_cmd.right_wheel = 0.0;
    motor_cmd.vibration = 1.0;

    while (std::abs(target_position - current_encoder_position_) > 0.01)
    {
      if (goal_handle->is_canceling())
      {
        motor_cmd.left_actuator = 0.0;
        motor_cmd.right_actuator = 0.0;
        motor_cmd.vibration = 0.0;
        motor_cmd_publisher_->publish(motor_cmd);
        return false;
      }

      motor_cmd.left_actuator = 1.0;
      motor_cmd.right_actuator = 1.0;
      motor_cmd_publisher_->publish(motor_cmd);

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    motor_cmd.left_actuator = 0.0;
    motor_cmd.right_actuator = 0.0;
    motor_cmd.vibration = 0.0;
    motor_cmd_publisher_->publish(motor_cmd);

    LOGGER_SUCCESS(
      this->get_logger(), "Bucket lowered to excavation position: %.2f", current_encoder_position_);
    return true;
  }

  /**
   * @brief Drives robot forward to excavate material.
   * @return true if successful, false if canceled
   */
  bool drive_forward(const std::shared_ptr<GoalHandleExcavation> goal_handle)
  {
    LOGGER_ACTION(this->get_logger(), "Driving forward to excavate...");

    auto motor_cmd = lunabot_msgs::msg::MotorCommands();
    motor_cmd.left_actuator = 0.0;
    motor_cmd.right_actuator = 0.0;
    motor_cmd.vibration = 1.0;

    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(forward_seconds))
    {
      if (goal_handle->is_canceling())
      {
        motor_cmd.left_wheel = 0.0;
        motor_cmd.right_wheel = 0.0;
        motor_cmd.vibration = 0.0;
        motor_cmd_publisher_->publish(motor_cmd);
        LOGGER_WARN(this->get_logger(), "Drive forward canceled");
        return false;
      }

      motor_cmd.left_wheel = 1.0;
      motor_cmd.right_wheel = -1.0;
      motor_cmd_publisher_->publish(motor_cmd);

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    motor_cmd.left_wheel = 0.0;
    motor_cmd.right_wheel = 0.0;
    motor_cmd.vibration = 0.0;
    motor_cmd_publisher_->publish(motor_cmd);

    LOGGER_SUCCESS(this->get_logger(), "Drive forward complete");
    return true;
  }

  /**
   * @brief Lifts bucket up after excavation.
   * @return true if successful, false if canceled
   */
  bool lift_bucket(const std::shared_ptr<GoalHandleExcavation> goal_handle)
  {
    LOGGER_ACTION(this->get_logger(), "Lifting bucket...");

    double target_position = travel_pos;

    auto motor_cmd = lunabot_msgs::msg::MotorCommands();
    motor_cmd.left_wheel = 0.0;
    motor_cmd.right_wheel = 0.0;
    motor_cmd.vibration = 0.0;

    while (std::abs(target_position - current_encoder_position_) > 0.01)
    {
      if (goal_handle->is_canceling())
      {
        motor_cmd.left_actuator = 0.0;
        motor_cmd.right_actuator = 0.0;
        motor_cmd.vibration = 0.0;
        motor_cmd_publisher_->publish(motor_cmd);
        return false;
      }

      motor_cmd.left_actuator = -1.0;
      motor_cmd.right_actuator = -1.0;
      motor_cmd_publisher_->publish(motor_cmd);

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    motor_cmd.left_actuator = 0.0;
    motor_cmd.right_actuator = 0.0;
    motor_cmd.vibration = 0.0;
    motor_cmd_publisher_->publish(motor_cmd);

    LOGGER_SUCCESS(
      this->get_logger(), "Bucket lifted to travel position: %.2f", current_encoder_position_);
    return true;
  }

  /**
   * @brief Executes the excavation action sequence.
   * @param goal_handle Handle for the action goal.
   */
  void execute(const std::shared_ptr<GoalHandleExcavation> goal_handle)
  {
    if (goal_active_)
    {
      LOGGER_WARN(this->get_logger(), "Excavation already in progress");
      auto result = std::make_shared<Excavation::Result>();
      result->success = false;
      result->message = "Excavation already in progress";
      goal_handle->abort(result);
      return;
    }

    goal_active_ = true;
    LOGGER_SUCCESS(this->get_logger(), "Starting excavation sequence");

    auto feedback = std::make_shared<Excavation::Feedback>();
    auto result = std::make_shared<Excavation::Result>();

    feedback->feedback_message = "Lowering bucket to excavation position";
    goal_handle->publish_feedback(feedback);
    if (!lower_bucket(goal_handle))
    {
      auto motor_cmd = lunabot_msgs::msg::MotorCommands();
      motor_cmd.left_wheel = 0.0;
      motor_cmd.right_wheel = 0.0;
      motor_cmd.left_actuator = 0.0;
      motor_cmd.right_actuator = 0.0;
      motor_cmd.vibration = 0.0;
      motor_cmd_publisher_->publish(motor_cmd);
      result->success = false;
      result->message = "Excavation canceled";
      goal_handle->canceled(result);
      LOGGER_WARN(this->get_logger(), "Excavation canceled during lower_bucket");
      goal_active_ = false;
      return;
    }

    feedback->feedback_message = "Driving forward to collect material";
    goal_handle->publish_feedback(feedback);
    if (!drive_forward(goal_handle))
    {
      auto motor_cmd = lunabot_msgs::msg::MotorCommands();
      motor_cmd.left_wheel = 0.0;
      motor_cmd.right_wheel = 0.0;
      motor_cmd.left_actuator = 0.0;
      motor_cmd.right_actuator = 0.0;
      motor_cmd.vibration = 0.0;
      motor_cmd_publisher_->publish(motor_cmd);
      result->success = false;
      result->message = "Excavation canceled";
      goal_handle->canceled(result);
      LOGGER_WARN(this->get_logger(), "Excavation canceled during drive_forward");
      goal_active_ = false;
      return;
    }

    feedback->feedback_message = "Lifting bucket to travel position";
    goal_handle->publish_feedback(feedback);
    if (!lift_bucket(goal_handle))
    {
      auto motor_cmd = lunabot_msgs::msg::MotorCommands();
      motor_cmd.left_wheel = 0.0;
      motor_cmd.right_wheel = 0.0;
      motor_cmd.left_actuator = 0.0;
      motor_cmd.right_actuator = 0.0;
      motor_cmd.vibration = 0.0;
      motor_cmd_publisher_->publish(motor_cmd);
      result->success = false;
      result->message = "Excavation canceled";
      goal_handle->canceled(result);
      LOGGER_WARN(this->get_logger(), "Excavation canceled during lift_bucket");
      goal_active_ = false;
      return;
    }

    result->success = true;
    result->message = "Excavation completed successfully";
    goal_handle->succeed(result);
    LOGGER_SUCCESS(this->get_logger(), "Excavation completed successfully");
    goal_active_ = false;
  }

  void encoder_position_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    current_encoder_position_ = msg->data;
  }

  void excavation_angle_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    excavation_angle_ = msg->data;
    LOGGER_INFO(this->get_logger(), "Excavation angle updated: %.2f", excavation_angle_);
  }

  rclcpp_action::Server<Excavation>::SharedPtr action_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr encoder_position_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr excavation_angle_subscriber_;
  rclcpp::Publisher<lunabot_msgs::msg::MotorCommands>::SharedPtr motor_cmd_publisher_;
  rclcpp::CallbackGroup::SharedPtr encoder_callback_group_;

  double current_encoder_position_ = 0.0;
  double excavation_angle_ = 1.59;
  bool goal_active_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<AssistedExcavationServer>();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
