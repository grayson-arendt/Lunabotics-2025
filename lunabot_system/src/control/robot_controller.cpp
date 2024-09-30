#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "SparkMax.hpp"

// Phoenix 6 headers
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp"
#include "ctre/phoenix6/configs/MotorOutputConfigs.hpp"

namespace phoenix6 = ctre::phoenix6;
using namespace phoenix6::hardware;

class RobotController : public rclcpp::Node {
public:
  RobotController()
      : Node("motor_controller"), manual_enabled_(true), robot_disabled_(false),
        left_wheel_motor_(1, "can0"), right_wheel_motor_(2, "can0"),
        lift_actuator_left_motor_("can0", 3),
        lift_actuator_right_motor_("can0", 4),
        tilt_actuator_left_motor_("can0", 5),
        tilt_actuator_right_motor_("can0", 6) {

    // Subscriptions
    velocity_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&RobotController::velocity_callback, this,
                  std::placeholders::_1));

    joystick_subscriber_ = create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&RobotController::joy_callback, this, std::placeholders::_1));

    // Setting parameters
    declare_and_get_parameters();
    apply_controller_mode();

    // Configure Talon FX motor settings
    configure_talon_fx_motors();

    RCLCPP_INFO(get_logger(),
                "\033[0;33mMANUAL CONTROL:\033[0m \033[1;32mENABLED\033[0m");
  }

private:
  void declare_and_get_parameters() {
    declare_parameter("xbox_mode", true);
    declare_parameter("ps4_mode", false);
    declare_parameter("switch_mode", false);
    declare_parameter("outdoor_mode", false);

    get_parameter("xbox_mode", xbox_mode_);
    get_parameter("ps4_mode", ps4_mode_);
    get_parameter("switch_mode", switch_mode_);
    get_parameter("outdoor_mode", outdoor_mode_);
  }

  void apply_controller_mode() {
    if (xbox_mode_)
      RCLCPP_INFO(get_logger(), "\033[0;35mXBOX MODE\033[0m");
    else if (ps4_mode_)
      RCLCPP_INFO(get_logger(), "\033[0;35mPS4 MODE\033[0m");
    else if (switch_mode_)
      RCLCPP_INFO(get_logger(), "\033[0;35mSWITCH MODE\033[0m");
    else
      RCLCPP_ERROR(get_logger(),
                   "NO CONTROLLER SELECTED, CHECK LAUNCH PARAMETERS");
  }

  void configure_talon_fx_motors() {
    phoenix6::configs::MotorOutputConfigs motorConfigs{};

    // Invert the right wheel motor to match the robot's design
    motorConfigs.Inverted = phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    right_wheel_motor_.GetConfigurator().Apply(motorConfigs);

    RCLCPP_INFO(get_logger(), "Talon FX motors configured.");
  }

  void control_robot() {
    if (home_button_)
      robot_disabled_ = true;

    lift_actuator_speed_ = (d_pad_vertical_ == 1.0)    ? -0.3
                           : (d_pad_vertical_ == -1.0) ? 0.3
                                                       : 0.0;
    tilt_actuator_speed_ = (right_joystick_y_ > 0.1)    ? -0.3
                           : (right_joystick_y_ < -0.1) ? 0.3
                                                        : 0.0;

    speed_multiplier_ =
        (x_button_ && y_button_) ? 0.3 : (x_button_ ? 0.1 : 0.6);

    if (switch_mode_) {
      left_speed_ = left_joystick_y_ - left_joystick_x_;
      right_speed_ = left_joystick_y_ + left_joystick_x_;
    } else {
      right_trigger_ = (1.0 - right_trigger_) / 2.0;
      left_trigger_ = (1.0 - left_trigger_) / 2.0;

      if (right_trigger_ != 0.0) {
        left_speed_ = right_trigger_ - left_joystick_x_;
        right_speed_ = right_trigger_ + left_joystick_x_;
      } else if (left_trigger_ != 0.0) {
        left_speed_ = -(left_trigger_ - left_joystick_x_);
        right_speed_ = -(left_trigger_ + left_joystick_x_);
      } else {
        left_speed_ = -left_joystick_x_;
        right_speed_ = left_joystick_x_;
      }
    }

    if (!robot_disabled_) {
      left_wheel_motor_.Set(phoenix6::signals::DutyCycle(left_speed_ * speed_multiplier_));
      right_wheel_motor_.Set(phoenix6::signals::DutyCycle(right_speed_ * speed_multiplier_));
    }

    lift_actuator_left_motor_.SetDutyCycle(lift_actuator_speed_);
    lift_actuator_right_motor_.SetDutyCycle(lift_actuator_speed_);
    tilt_actuator_left_motor_.SetDutyCycle(tilt_actuator_speed_);
    tilt_actuator_right_motor_.SetDutyCycle(tilt_actuator_speed_);
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    auto clock = rclcpp::Clock();

    share_button_ = get_button(joy_msg, {9, 8, 6});
    menu_button_ = get_button(joy_msg, {10, 9, 7});
    home_button_ = get_button(joy_msg, {11, 10, 8});
    a_button_ = get_button(joy_msg, {1, 0, 0});
    b_button_ = get_button(joy_msg, {0, 2, 1});
    x_button_ = get_button(joy_msg, {2, 3, 2});
    y_button_ = get_button(joy_msg, {3, 2, 3});
    left_bumper_ = get_button(joy_msg, {4, 4, 4});
    right_bumper_ = get_button(joy_msg, {5, 5, 5});

    d_pad_horizontal_ = get_axis(joy_msg, {4, 6, 6});
    d_pad_vertical_ = get_axis(joy_msg, {5, 7, 7});

    if (share_button_) {
      manual_enabled_ = true;
      RCLCPP_INFO_THROTTLE(
          get_logger(), clock, 1000,
          "\033[0;33mMANUAL CONTROL: \033[0m\033[1;32mENABLED\033[0m");
    }

    if (menu_button_) {
      manual_enabled_ = false;
      RCLCPP_INFO_THROTTLE(
          get_logger(), clock, 1000,
          "\033[0;36mAUTONOMOUS CONTROL: \033[0m\033[1;32mENABLED\033[0m");
    }

    if (manual_enabled_) {
      left_joystick_x_ = joy_msg->axes[0];
      left_joystick_y_ = joy_msg->axes[1];
      right_joystick_y_ = joy_msg->axes[4];
      left_trigger_ = joy_msg->axes[2];
      right_trigger_ = joy_msg->axes[5];

      SparkFlex::Heartbeat();

      control_robot();
    }
  }

  void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr velocity_msg) {
    if (!manual_enabled_ && navigation_enabled_) {
      SparkFlex::Heartbeat();
      double linear_velocity = velocity_msg->linear.x;
      double angular_velocity = velocity_msg->angular.z;
      double wheel_radius = outdoor_mode_ ? 0.2 : 0.1016;
      double wheel_distance = 0.5;

      double velocity_left_cmd =
          0.1 * (linear_velocity - angular_velocity * wheel_distance / 2.0) /
          wheel_radius;
      double velocity_right_cmd =
          0.1 * (linear_velocity + angular_velocity * wheel_distance / 2.0) /
          wheel_radius;

      left_wheel_motor_.Set(phoenix6::signals::DutyCycle(velocity_left_cmd));
      right_wheel_motor_.Set(phoenix6::signals::DutyCycle(velocity_right_cmd));
    }
  }

  int get_button(const sensor_msgs::msg::Joy::SharedPtr &joy_msg,
                 const std::initializer_list<int> &mappings) {
    size_t index = xbox_mode_ ? 2 : ps4_mode_ ? 1 : 0;
    return joy_msg->buttons[*(mappings.begin() + index)];
  }

  double get_axis(const sensor_msgs::msg::Joy::SharedPtr &joy_msg,
                  const std::initializer_list<int> &mappings) {
    size_t index = xbox_mode_ ? 2 : ps4_mode_ ? 1 : 0;
    return joy_msg->axes[*(mappings.begin() + index)];
  }

  // Member variables
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber_;

  bool manual_enabled_;
  bool robot_disabled_;
  bool xbox_mode_, ps4_mode_, switch_mode_, outdoor_mode_;
  bool share_button_, menu_button_, home_button_, a_button_, b_button_, x_button_, y_button_;
  bool left_bumper_, right_bumper_;
  double left_speed_, right_speed_, left_joystick_x_, left_joystick_y_, right_joystick_y_;
  double d_pad_horizontal_, d_pad_vertical_;
  double lift_actuator_speed_, tilt_actuator_speed_;
  double left_trigger_, right_trigger_;
  double speed_multiplier_;

  // Motor objects
  TalonFX left_wheel_motor_;
  TalonFX right_wheel_motor_;
  SparkMax lift_actuator_left_motor_;
  SparkMax lift_actuator_right_motor_;
  SparkMax tilt_actuator_left_motor_;
  SparkMax tilt_actuator_right_motor_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
