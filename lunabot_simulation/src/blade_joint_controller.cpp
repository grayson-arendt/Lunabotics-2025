/**
 * @file blade_joint_controller.cpp
 * @author Grayson Arendt
 * @date 9/18/2024
 */

#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

/**
 * @class BladeJointController
 * @brief Controls the blade joint pitch angle using joystick input.
 */
class BladeJointController : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for BladeJointController.
   */
  BladeJointController() : Node("blade_joint_controller"), pitch_angle_(0.0)
  {
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&BladeJointController::joy_callback, this,
                  std::placeholders::_1));

    joint_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/position_controller/commands", 10);

    max_pitch_angle_ = 0.75;
    min_pitch_angle_ = -0.75;
  }

private:
  /**
   * @brief Callback for processing joystick input and publishing blade joint
   * commands.
   * @param msg The joystick input message.
   */
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    float y_axis_value = msg->axes[4];
    float scaling_factor = 0.025;

    pitch_angle_ += -y_axis_value * scaling_factor;
    pitch_angle_ = std::clamp(pitch_angle_, min_pitch_angle_, max_pitch_angle_);

    std_msgs::msg::Float64MultiArray commands;
    commands.data.push_back(pitch_angle_);

    joint_publisher_->publish(commands);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      joint_publisher_;

  float pitch_angle_, max_pitch_angle_, min_pitch_angle_;
};

/**
 * @brief Main function.
 * Initializes and spins the BladeJointController node.
 */
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BladeJointController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
