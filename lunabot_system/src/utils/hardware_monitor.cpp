/**
 * @file hardware_monitor.cpp
 * @author Grayson Arendt
 * @date 9/18/2024
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

/**
 * @class HardwareMonitor
 * @brief Monitors the hardware connections and status of sensors (D456, D455, and LIDARs).
 */
class HardwareMonitor : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for HardwareMonitor.
   */
  HardwareMonitor() : Node("hardware_monitor"), d456_valid_(false), d455_valid_(false), lidar1_valid_(false), lidar2_valid_(false)
  {
    d456_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
        "d456/color/image_raw", rclcpp::QoS(10).reliable(),
        [this](const sensor_msgs::msg::Image::SharedPtr) { reset_timer(d456_timer_, d456_valid_); });

    d455_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
        "d455/color/image_raw", rclcpp::QoS(10).reliable(),
        [this](const sensor_msgs::msg::Image::SharedPtr) { reset_timer(d455_timer_, d455_valid_); });

    lidar1_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::QoS(10).reliable(),
        [this](const sensor_msgs::msg::LaserScan::SharedPtr) { reset_timer(lidar1_timer_, lidar1_valid_); });

    lidar2_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan2", rclcpp::QoS(10).reliable(),
        [this](const sensor_msgs::msg::LaserScan::SharedPtr) { reset_timer(lidar2_timer_, lidar2_valid_); });

    d456_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() { check_connection(d456_valid_, "D456"); });
    d455_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() { check_connection(d455_valid_, "D455"); });
    lidar1_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() { check_connection(lidar1_valid_, "LIDAR A3"); });
    lidar2_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() { check_connection(lidar2_valid_, "LIDAR S2L"); });

    all_sensors_timer_ = create_wall_timer(std::chrono::seconds(1), [this]()
                                           {
      if (d456_valid_ && d455_valid_ && lidar1_valid_ && lidar2_valid_) {
        RCLCPP_INFO(get_logger(), "\033[38;5;208mSENSOR STATUS:\033[0m \033[1;32mGOOD\033[0m");
      } });
  }

  /**
   * @brief Resets the timer and sets the validity flag to true.
   * @param timer The timer to reset.
   * @param valid_flag The validity flag to set.
   */
  void reset_timer(rclcpp::TimerBase::SharedPtr &timer, bool &valid_flag)
  {
    timer->reset();
    valid_flag = true;
  }

  /**
   * @brief Checks the connection status and logs a warning if no connection is detected.
   * @param valid_flag The validity flag indicating the connection status.
   * @param sensor_name The name of the sensor.
   */
  void check_connection(bool &valid_flag, const std::string &sensor_name)
  {
    if (!valid_flag)
    {
      RCLCPP_WARN(get_logger(), "\033[0;36m%s:\033[0m \033[1;31mNO CONNECTION, CHECK CABLE\033[0m", sensor_name.c_str());
    }
    valid_flag = false;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr d456_subscriber_, d455_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar1_subscriber_, lidar2_subscriber_;
  rclcpp::TimerBase::SharedPtr d456_timer_, d455_timer_, lidar1_timer_, lidar2_timer_, all_sensors_timer_;
  bool d456_valid_, d455_valid_, lidar1_valid_, lidar2_valid_;
};

/**
 * @brief Main function.
 * Initializes and spins the HardwareMonitor node.
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HardwareMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
