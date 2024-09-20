#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class OdomTfPublisher : public rclcpp::Node
{
public:
    OdomTfPublisher()
        : Node("odom_tf_publisher")
    {
        // Subscriber to the /odometry/filtered topic
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&OdomTfPublisher::odomCallback, this, std::placeholders::_1));

        // Initialize the TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    // Callback for the odometry subscriber
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Create a transform stamped message
        geometry_msgs::msg::TransformStamped transformStamped;

        // Set the header frame id (source frame) as "odom"
        transformStamped.header.stamp = msg->header.stamp;
        transformStamped.header.frame_id = "odom";

        // Set the child frame id (target frame) as "base_link"
        transformStamped.child_frame_id = "base_link";

        // Set the translation (x, y, z) from the odometry message
        transformStamped.transform.translation.x = msg->pose.pose.position.x;
        transformStamped.transform.translation.y = msg->pose.pose.position.y;
        transformStamped.transform.translation.z = msg->pose.pose.position.z;

        // Set the rotation (quaternion) from the odometry message
        transformStamped.transform.rotation = msg->pose.pose.orientation;

        // Publish the transform
        tf_broadcaster_->sendTransform(transformStamped);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomTfPublisher>());
    rclcpp::shutdown();
    return 0;
}
