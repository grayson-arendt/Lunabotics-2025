/**
 * @file localization_server.cpp
 * @author Grayson Arendt
 * @date 9/30/2024
 */

#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "cv_bridge/cv_bridge.h"

#include "lunabot_system/action/localization.hpp"

/**
 * @class LocalizationServer
 * @brief Aligns the robot to an AprilTag and provides localization feedback for pose estimation.
 */
class LocalizationServer : public rclcpp::Node
{
public:
    using Localization = lunabot_system::action::Localization;
    using GoalHandleLocalization = rclcpp_action::ServerGoalHandle<Localization>;

    /**
     * @brief Constructor for LocalizationServer class.
     */
    LocalizationServer() : Node("localization_server"), tag_detected_(false), aligned_(false), lateral_distance_(0.0), depth_distance_(0.0), alignment_started_(false), goal_received_(false)
    {
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "d455/color/image_raw", 10, std::bind(&LocalizationServer::detect_apriltag, this, std::placeholders::_1));

        overlay_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("apriltag/overlay_image", 10);
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        action_server_ = rclcpp_action::create_server<Localization>(
            this,
            "localization_action",
            std::bind(&LocalizationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&LocalizationServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&LocalizationServer::handle_accepted, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&LocalizationServer::align_robot, this));
    }

private:
    /**
     * @brief Handles incoming localization goal.
     */
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const Localization::Goal>)
    {
        goal_received_ = true;
        alignment_started_ = false;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /**
     * @brief Handles localization goal cancellation.
     */
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleLocalization>)
    {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    /**
     * @brief Handles accepted localization goal.
     */
    void handle_accepted(const std::shared_ptr<GoalHandleLocalization> goal_handle)
    {
        std::thread{std::bind(&LocalizationServer::execute, this, goal_handle)}.detach();
    }

    /**
     * @brief Executes localization process, aligning robot to AprilTag.
     */
    void execute(const std::shared_ptr<GoalHandleLocalization> goal_handle)
    {
        auto result = std::make_shared<Localization::Result>();

        while (!aligned_ && rclcpp::ok())
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (aligned_)
        {
            RCLCPP_INFO(this->get_logger(), "\033[1;32m LOCALIZATION SUCCESS!\033[0m");
            result->success = true;
            result->x = -lateral_distance_;
            result->y = depth_distance_;
            result->yaw = tag_yaw;
            goal_handle->succeed(result);
        }
        else
        {
            result->success = false;
            goal_handle->abort(result);
        }
    }

    /**
     * @brief Detects AprilTags and calculates distances and yaw.
     * @param inputImage The input image from the camera.
     */
    void detect_apriltag(const sensor_msgs::msg::Image::SharedPtr inputImage)
    {
        try
        {
            auto currentImage_ptr = cv_bridge::toCvCopy(inputImage, inputImage->encoding);
            auto outputImage = currentImage_ptr->image.clone();

            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners;

            cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << 383.4185742519996, 0, 309.4326377845713, 0, 385.0909007102088, 240.749949733094, 0, 0, 1);
            cv::Mat distortionCoefficients = (cv::Mat1d(1, 5) << -0.06792929080519726, 0.08058277259698843, -0.001690544521662593, -0.0008235437909836152, -0.04417756393089296);

            std::vector<cv::Vec3d> rvecs, tvecs;
            auto parameters = cv::aruco::DetectorParameters::create();
            auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);

            cv::aruco::detectMarkers(currentImage_ptr->image, dictionary, markerCorners, markerIds, parameters);
            cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.3125, cameraMatrix, distortionCoefficients, rvecs, tvecs);

            if (!markerIds.empty())
            {
                tag_detected_ = true;
                calculate_distances(tvecs[0], lateral_distance_, depth_distance_);
                calculate_yaw(rvecs[0], tag_yaw);
                tag_yaw = normalize_angle(tag_yaw);

                cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
                cv::aruco::drawAxis(outputImage, cameraMatrix, distortionCoefficients, rvecs[0], tvecs[0], 0.1);

                auto msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", outputImage).toImageMsg();
                overlay_publisher_->publish(*msg_);
            }
            else
            {
                tag_detected_ = false;
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "ERROR PROCESSING IMAGE: %s", e.what());
        }
    }

    /**
     * @brief Aligns robot to face the detected AprilTag.
     */
    void align_robot()
    {
        if (!alignment_started_)
        {
            alignment_start_time_ = this->now();
            alignment_started_ = true;
        }

        double elapsed_time = (this->now() - alignment_start_time_).seconds();

        if (elapsed_time > 60.0 && !aligned_)
        {
            geometry_msgs::msg::Twist twist;
            twist.angular.z = 0.0;
            cmd_vel_publisher_->publish(twist);
            aligned_ = false;
            return;
        }

        if (!tag_detected_ && !aligned_)
        {
            geometry_msgs::msg::Twist twist;
            twist.angular.z = 0.4;
            cmd_vel_publisher_->publish(twist);
        }
        else if (tag_detected_ && !aligned_)
        {
            geometry_msgs::msg::Twist twist;
            double yaw_error = normalize_angle(tag_yaw);

            if (std::abs(yaw_error) > 0.025)
                twist.angular.z = 0.125;
            else
            {
                twist.angular.z = 0.0;
                aligned_ = true;
            }

            cmd_vel_publisher_->publish(twist);
        }

        else {
            aligned_ = true;
        }
    }

    /**
     * @brief Normalizes angle to [-pi, pi].
     * @param angle The input angle.
     * @return Normalized angle.
     */
    double normalize_angle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    /**
     * @brief Calculates lateral and depth distances.
     * @param tvec The translation vector.
     * @param lateral_distance_ Lateral distance (X).
     * @param depth_distance_ Depth distance (Z).
     */
    void calculate_distances(const cv::Vec3d &tvec, double &lateral_distance_, double &depth_distance_)
    {
        lateral_distance_ = tvec[0];
        depth_distance_ = tvec[2];
    }

    /**
     * @brief Extracts yaw angle from the rotation vector.
     * @param rvec The rotation vector.
     * @param tag_yaw The extracted yaw angle.
     */
    void calculate_yaw(const cv::Vec3d &rvec, double &tag_yaw)
    {
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        tag_yaw = asin(rotation_matrix.at<double>(2, 0));
    }

    bool tag_detected_, aligned_, goal_received_, alignment_started_;
    double lateral_distance_, depth_distance_, tag_yaw;
    rclcpp::Time alignment_start_time_;

    rclcpp_action::Server<Localization>::SharedPtr action_server_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

/**
 * @brief Main function.
 * Initializes and runs the LocalizationServer node.
 */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationServer>());
    rclcpp::shutdown();
    return 0;
}