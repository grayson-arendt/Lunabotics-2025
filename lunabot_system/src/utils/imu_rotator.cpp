/**
 * @file imu_rotator.cpp
 * @author Grayson Arendt
 * @date 9/18/2024
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

/**
 * @class IMURotator
 * @brief Converts IMU data from NED to ENU frame for both D455 and D456 cameras.
 */
class IMURotator : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for IMURotator.
   */
  IMURotator() : Node("imu_rotator")
  {
    d455_imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "d455/imu", 10, std::bind(&IMURotator::d455_imu_callback, this, std::placeholders::_1));

    d456_imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "d456/imu", 10, std::bind(&IMURotator::d456_imu_callback, this, std::placeholders::_1));

    d455_imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("d455/imu/data_raw", 10);
    d456_imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("d456/imu/data_raw", 10);
  }

private:
  /**
   * @brief Callback function for transforming and publishing D455 IMU data from NED to ENU frame.
   * @param msg The received IMU message.
   */
  void d455_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    auto transformed_msg = std::make_shared<sensor_msgs::msg::Imu>(*msg);
    transform_imu_data(msg, transformed_msg);
    d455_imu_publisher_->publish(*transformed_msg);
  }

  /**
   * @brief Callback function for transforming and publishing D456 IMU data from NED to ENU frame.
   * @param msg The received IMU message.
   */
  void d456_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    auto transformed_msg = std::make_shared<sensor_msgs::msg::Imu>(*msg);
    transform_imu_data(msg, transformed_msg);
    d456_imu_publisher_->publish(*transformed_msg);
  }

  /**
   * @brief Transforms IMU data from NED to ENU frame.
   * @param msg The original IMU message.
   * @param transformed_msg The transformed IMU message.
   */
  void transform_imu_data(const sensor_msgs::msg::Imu::SharedPtr msg, std::shared_ptr<sensor_msgs::msg::Imu> &transformed_msg)
  {
    transformed_msg->linear_acceleration.x = msg->linear_acceleration.y;
    transformed_msg->linear_acceleration.y = msg->linear_acceleration.x;
    transformed_msg->linear_acceleration.z = -msg->linear_acceleration.z;

    transformed_msg->angular_velocity.x = msg->angular_velocity.y;
    transformed_msg->angular_velocity.y = msg->angular_velocity.x;
    transformed_msg->angular_velocity.z = -msg->angular_velocity.z;

    tf2::Quaternion quat_ned(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m_ned(quat_ned);

    double roll, pitch, yaw;
    m_ned.getRPY(roll, pitch, yaw);

    yaw = -yaw;

    tf2::Quaternion quat_enu;
    quat_enu.setRPY(roll, pitch, yaw);

    transformed_msg->orientation.x = quat_enu.x();
    transformed_msg->orientation.y = quat_enu.y();
    transformed_msg->orientation.z = quat_enu.z();
    transformed_msg->orientation.w = quat_enu.w();
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr d455_imu_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr d456_imu_subscriber_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr d455_imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr d456_imu_publisher_;
};

/**
 * @brief Main function.
 * Initializes and spins the IMURotator node.
 */
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMURotator>());
  rclcpp::shutdown();
  return 0;
}
