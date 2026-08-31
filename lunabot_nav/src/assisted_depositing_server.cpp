/**
 * @file assisted_depositing_server.cpp
 * @author Grayson Arendt
 * @date 02/22/2026
 */

#include "lunabot_logger/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "lunabot_msgs/action/depositing.hpp"
#include "lunabot_msgs/msg/motor_commands.hpp"
#include <std_msgs/msg/float64.hpp>

#include <chrono>
#include <memory>
#include <thread>

static constexpr double deposit_pos = 0.0;
static constexpr double travel_pos = 1.0;
static constexpr double deposit_seconds = 5.0;

/**
 * @class AssistedDepositingServer
 * @brief Assisted depositing server for teleop that controls bucket actuators for depositing
 * sequence.
 */
class AssistedDepositingServer : public rclcpp::Node
{
public:
  using Depositing = lunabot_msgs::action::Depositing;
  using GoalHandleDepositing = rclcpp_action::ServerGoalHandle<Depositing>;

  /**
   * @brief Constructor for the AssistedDepositingServer class.
   */
  AssistedDepositingServer() : Node("assisted_depositing_server")
  {
    encoder_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    action_server_ = rclcpp_action::create_server<Depositing>(
      this, "assisted_depositing_action",
      [this](const auto &, const auto &) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const auto &) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](const auto goal_handle) {
        std::thread{[this, goal_handle]() { execute(goal_handle); }}.detach();
      });

    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = encoder_callback_group_;

    encoder_position_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "bucket_angle", 10,
      std::bind(&AssistedDepositingServer::encoder_position_callback, this, std::placeholders::_1),
      sub_options);

    motor_cmd_publisher_ =
      this->create_publisher<lunabot_msgs::msg::MotorCommands>("/motor_commands", 10);

    LOGGER_SUCCESS(this->get_logger(), "Assisted depositing server initialized");
  }

private:
  /**
   * @brief Lifts bucket to deposit position.
   * @return true if successful, false if canceled
   */
  bool lift_bucket(const std::shared_ptr<GoalHandleDepositing> goal_handle)
  {
    LOGGER_ACTION(this->get_logger(), "Lifting bucket to deposit...");

    double target_position = deposit_pos;

    auto motor_cmd = lunabot_msgs::msg::MotorCommands();
    motor_cmd.left_wheel = 0.0;
    motor_cmd.right_wheel = 0.0;
    motor_cmd.vibration = 0.0;

    while (std::abs(current_encoder_position_ - target_position) > 0.2)
    {
      if (goal_handle->is_canceling())
      {
        motor_cmd.left_actuator = 0.0;
        motor_cmd.right_actuator = 0.0;
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
    motor_cmd_publisher_->publish(motor_cmd);

    LOGGER_ACTION(this->get_logger(), "Vibrating bucket for %.2f seconds...", deposit_seconds);

    auto start = std::chrono::steady_clock::now();

    bool vibration_on = true;

    while (vibration_on)
    {
      if (goal_handle->is_canceling())
      {
        motor_cmd.vibration = 0.0;
        motor_cmd_publisher_->publish(motor_cmd);
        return false;
      }

      motor_cmd.vibration = 1.0;
      motor_cmd_publisher_->publish(motor_cmd);

      auto end = std::chrono::steady_clock::now();

      std::chrono::duration<double> elapsed_seconds = end - start;

      if (elapsed_seconds.count() >= deposit_seconds)
      {
        vibration_on = false;
        motor_cmd.vibration = 0.0;
        motor_cmd_publisher_->publish(motor_cmd);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return true;
  }

  /**
   * @brief Lowers bucket back to travel position.
   * @return true if successful, false if canceled
   */
  bool lower_bucket(const std::shared_ptr<GoalHandleDepositing> goal_handle)
  {
    LOGGER_ACTION(this->get_logger(), "Lowering bucket to travel position...");

    double target_position = travel_pos;

    auto motor_cmd = lunabot_msgs::msg::MotorCommands();
    motor_cmd.left_wheel = 0.0;
    motor_cmd.right_wheel = 0.0;
    motor_cmd.vibration = 0.0;

    while (std::abs(current_encoder_position_ - target_position) > 0.2)
    {
      if (goal_handle->is_canceling())
      {
        motor_cmd.left_actuator = 0.0;
        motor_cmd.right_actuator = 0.0;
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
    motor_cmd_publisher_->publish(motor_cmd);

    return true;
  }

  /**
   * @brief Executes the depositing action sequence.
   * @param goal_handle Handle for the action goal.
   */
  void execute(const std::shared_ptr<GoalHandleDepositing> goal_handle)
  {
    if (goal_active_)
    {
      LOGGER_WARN(this->get_logger(), "Depositing already in progress");
      auto result = std::make_shared<Depositing::Result>();
      result->success = false;
      result->message = "Depositing already in progress";
      goal_handle->abort(result);
      return;
    }

    goal_active_ = true;
    LOGGER_SUCCESS(this->get_logger(), "Starting depositing sequence");

    auto feedback = std::make_shared<Depositing::Feedback>();
    auto result = std::make_shared<Depositing::Result>();

    try
    {
      feedback->feedback_message = "Lifting bucket to deposit position";
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
        result->message = "Depositing canceled";
        goal_handle->canceled(result);
        LOGGER_WARN(this->get_logger(), "Depositing canceled");
        goal_active_ = false;
        return;
      }

      feedback->feedback_message = "Returning bucket to home position";
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
        result->message = "Depositing canceled";
        goal_handle->canceled(result);
        LOGGER_WARN(this->get_logger(), "Depositing canceled");
        goal_active_ = false;
        return;
      }

      result->success = true;
      result->message = "Depositing completed successfully";
      goal_handle->succeed(result);
      LOGGER_SUCCESS(this->get_logger(), "Depositing completed successfully");
    } catch (const std::exception & e)
    {
      auto motor_cmd = lunabot_msgs::msg::MotorCommands();
      motor_cmd.left_wheel = 0.0;
      motor_cmd.right_wheel = 0.0;
      motor_cmd.left_actuator = 0.0;
      motor_cmd.right_actuator = 0.0;
      motor_cmd.vibration = 0.0;
      motor_cmd_publisher_->publish(motor_cmd);
      result->success = false;
      result->message = std::string("Depositing failed: ") + e.what();
      goal_handle->abort(result);
      LOGGER_FAILURE(this->get_logger(), "Depositing failed: %s", e.what());
    }

    goal_active_ = false;
  }

  void encoder_position_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    current_encoder_position_ = msg->data;
  }

  rclcpp_action::Server<Depositing>::SharedPtr action_server_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr encoder_position_subscriber_;
  rclcpp::Publisher<lunabot_msgs::msg::MotorCommands>::SharedPtr motor_cmd_publisher_;
  rclcpp::CallbackGroup::SharedPtr encoder_callback_group_;

  double current_encoder_position_ = 0.0;
  bool goal_active_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<AssistedDepositingServer>();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
