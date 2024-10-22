/**
 * @file navigation_client.cpp
 * @author Grayson Arendt
 * @date 10/2/2024
 */

#include <chrono>
#include <cmath>

#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "lunabot_system/action/localization.hpp"

/**
 * @class NavigationClient
 * @brief Sends navigation goals, handles received localization data, and controls robot movement based on odometry
 * feedback.
 */
class NavigationClient : public rclcpp::Node
{
  public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using Localization = lunabot_system::action::Localization;
    using GoalHandleLocalization = rclcpp_action::ClientGoalHandle<Localization>;

    /**
     * @brief Constructor for the NavigationClient class.
     */
    NavigationClient()
        : Node("navigator_client"), goal_reached_(false), localization_done_(false), current_goal_index_(0)
    {
        nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        localization_client_ = rclcpp_action::create_client<Localization>(this, "localization_action");

        blade_position_publisher_ =
            this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);

        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&NavigationClient::localize_and_send_goal, this));

        // Define the waypoints
        setup_waypoints();
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
            send_goal_options.result_callback =
                std::bind(&NavigationClient::handle_localization_result, this, std::placeholders::_1);
            localization_client_->async_send_goal(goal_msg, send_goal_options);
        }

        if (localization_done_ && current_goal_index_ < waypoints_.size())
        {
            send_navigation_goal();
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
     * @brief Sends the navigation goal.
     */
    void send_navigation_goal()
    {
        if (!this->nav_to_pose_client_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "NAV2 ACTION SERVER NOT AVAILABLE.");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = waypoints_[current_goal_index_];

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            std::bind(&NavigationClient::goal_result_callback, this, std::placeholders::_1);

        this->nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    }

    /**
     * @brief Callback for the result of the navigation goal.
     * @param result The result of the goal execution.
     */
    void goal_result_callback(const GoalHandleNavigate::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "GOAL %ld REACHED!", current_goal_index_);

            current_goal_index_++;

            if (current_goal_index_ == 1)
            {
                RCLCPP_INFO_ONCE(this->get_logger(), "\033[1;32mROTATING BLADE...!\033[0m");
                std_msgs::msg::Float64MultiArray commands;
                commands.data.push_back(0.1);

                blade_position_publisher_->publish(commands);
            }

            else if (current_goal_index_ >= waypoints_.size())
            {
                goal_reached_ = true;
                RCLCPP_INFO_ONCE(this->get_logger(), "\033[1;32mALL GOALS SUCCESSFULLY REACHED!\033[0m");
            }
        }
    }

    /**
     * @brief Sets up the list of waypoints (goals) for the robot.
     */
    void setup_waypoints()
    {
        geometry_msgs::msg::PoseStamped pose1;
        pose1.header.stamp = this->now();
        pose1.header.frame_id = "map";
        pose1.pose.position.x = initial_x_ + 3.3;
        pose1.pose.position.y = initial_y_ + 3.2;
        pose1.pose.orientation.z = 0.707;
        pose1.pose.orientation.w = 0.707;
        waypoints_.push_back(pose1);

        geometry_msgs::msg::PoseStamped pose2;
        pose2.header.stamp = this->now();
        pose2.header.frame_id = "map";
        pose2.pose.position.x = 4.03;
        pose2.pose.position.y = 0.6;
        pose2.pose.orientation.z = 0.707;
        pose2.pose.orientation.w = 0.707;
        waypoints_.push_back(pose2);
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
    rclcpp_action::Client<Localization>::SharedPtr localization_client_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr blade_position_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool goal_reached_, localization_done_;
    double initial_x_, initial_y_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t current_goal_index_;
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
