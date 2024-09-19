#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class TopicRemap : public rclcpp::Node {
public:
  TopicRemap() : Node("topic_remap") {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/diff_drive_controller/odom", 10,
        std::bind(&TopicRemap::odom_callback, this, std::placeholders::_1));

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&TopicRemap::cmd_vel_callback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diff_drive_controller/cmd_vel_unstamped", 10);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_pub_->publish(*msg);
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    cmd_vel_pub_->publish(*msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicRemap>());
  rclcpp::shutdown();
  return 0;
}
