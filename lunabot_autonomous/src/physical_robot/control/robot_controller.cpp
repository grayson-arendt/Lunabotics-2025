#include <unistd.h>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <limits>

#include "lunabot_autonomous/msg/control.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#define Phoenix_No_WPI
#include "ctre/Phoenix.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

/**
 * @class RobotController
 * @brief A class for controlling the robot using a controller (XBox One, PS4, or Nintendo Switch) and autonomous commands.
 * @details
 * The Nintendo Switch controller triggers act as buttons instead of providing variable input, so the mode will drive the robot
 * based off of the left joystick axes instead.
 * @author Grayson Arendt
 */
class RobotController : public rclcpp::Node
{
public:
    RobotController() : Node("motor_controller"), manual_enabled_(true), robot_disabled_(false)
    {
        // Subscriptions
        velocity_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&RobotController::velocity_callback, this, std::placeholders::_1));

        control_subscriber_ = create_subscription<lunabot_autonomous::msg::Control>(
            "control", 10, std::bind(&RobotController::control_callback, this, std::placeholders::_1));

        joystick_subscriber_ = create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&RobotController::joy_callback, this, std::placeholders::_1));

        // Setting parameters
        declare_and_get_parameters();
        apply_controller_mode();

        right_wheel_motor_.SetInverted(true);
        RCLCPP_INFO(get_logger(), "\033[0;33mMANUAL CONTROL:\033[0m \033[1;32mENABLED\033[0m");
    }

private:
    /**
     * @brief Declares and retrieves parameters from the parameter server. Default to Xbox controller.
     */
    void declare_and_get_parameters()
    {
        declare_parameter("xbox_mode", true);
        declare_parameter("ps4_mode", false);
        declare_parameter("switch_mode", false);
        declare_parameter("outdoor_mode", false);

        get_parameter("xbox_mode", xbox_mode_);
        get_parameter("ps4_mode", ps4_mode_);
        get_parameter("switch_mode", switch_mode_);
        get_parameter("outdoor_mode", outdoor_mode_);
    }

    /**
     * @brief Applies the controller mode based on the parameters.
     */
    void apply_controller_mode()
    {
        if (xbox_mode_)
            RCLCPP_INFO(get_logger(), "\033[0;35mXBOX MODE\033[0m");
        else if (ps4_mode_)
            RCLCPP_INFO(get_logger(), "\033[0;35mPS4 MODE\033[0m");
        else if (switch_mode_)
            RCLCPP_INFO(get_logger(), "\033[0;35mSWITCH MODE\033[0m");
        else
            RCLCPP_ERROR(get_logger(), "NO CONTROLLER SELECTED, CHECK LAUNCH PARAMETERS");
    }

    /**
     * @brief Sets motor speeds and controls robot based on joystick input.
     */
    void control_robot()
    {
        // Disable the robot if home button is pressed
        if (home_button_)
            robot_disabled_ = true;

        // Set speeds for accessories
        vibrator_speed_ = a_button_ ? 0.8 : 0.0;
        door_speed_ = left_bumper_ ? -0.5 : right_bumper_ ? 0.5
                                                          : 0.0;
        actuator_speed_ = (d_pad_vertical_ == 1.0) ? -0.3 : (d_pad_vertical_ == -1.0) ? 0.3
                                                                                      : 0.0;

        // Handle speed reduction when trencher is active
        trencher_speed_ = x_button_ ? -0.6 : 0.0;
        speed_multiplier_ = (x_button_ && y_button_) ? 0.3 : (x_button_ ? 0.1 : 0.6);

        // Handle driving logic based on controller type
        if (switch_mode_)
        {
            left_speed_ = left_joystick_y_ - left_joystick_x_;
            right_speed_ = left_joystick_y_ + left_joystick_x_;
        }
        else
        {
            right_trigger_ = (1.0 - right_trigger_) / 2.0;
            left_trigger_ = (1.0 - left_trigger_) / 2.0;

            if (right_trigger_ != 0.0)
            {
                left_speed_ = right_trigger_ - left_joystick_x_;
                right_speed_ = right_trigger_ + left_joystick_x_;
            }
            else if (left_trigger_ != 0.0)
            {
                left_speed_ = -(left_trigger_ - left_joystick_x_);
                right_speed_ = -(left_trigger_ + left_joystick_x_);
            }
            else
            {
                left_speed_ = -left_joystick_x_;
                right_speed_ = left_joystick_x_;
            }
        }

        // Set motor output speeds
        left_wheel_motor_.Set(ControlMode::PercentOutput, left_speed_ * speed_multiplier_);
        right_wheel_motor_.Set(ControlMode::PercentOutput, right_speed_ * speed_multiplier_);
        actuator_left_motor_.Set(ControlMode::PercentOutput, actuator_speed_);
        actuator_right_motor_.Set(ControlMode::PercentOutput, actuator_speed_);
        trencher_motor_.Set(ControlMode::PercentOutput, trencher_speed_);
        door_motor_.Set(ControlMode::PercentOutput, door_speed_);
        vibrator_motor_.Set(ControlMode::PercentOutput, vibrator_speed_);
    }

    /**
     * @brief Processes joystick input.
     */
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
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

        // Enable or disable manual control
        if (share_button_)
        {
            manual_enabled_ = true;
            RCLCPP_INFO_THROTTLE(get_logger(), clock, 1000, "\033[0;33mMANUAL CONTROL: \033[0m\033[1;32mENABLED\033[0m");
        }

        if (menu_button_)
        {
            manual_enabled_ = false;
            RCLCPP_INFO_THROTTLE(get_logger(), clock, 1000, "\033[0;36mAUTONOMOUS CONTROL: \033[0m\033[1;32mENABLED\033[0m");
        }

        // Driving the robot with controller
        if (manual_enabled_)
        {
            left_joystick_x_ = joy_msg->axes[0];
            left_joystick_y_ = joy_msg->axes[1];
            left_trigger_ = joy_msg->axes[2];
            right_trigger_ = joy_msg->axes[5];

            // Enable motor control
            if (robot_disabled_)
                RCLCPP_ERROR(get_logger(), "\033[0;31mROBOT DISABLED\033[0m");
            else
                ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);

            // Sets motor speeds to drive robot and run accessories
            control_robot();
        }
    }

    /**
     * @brief Callback for velocity messages.
     */
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr velocity_msg)
    {
        if (!manual_enabled_)
        {
            ctre::phoenix::unmanaged::Unmanaged::FeedEnable(1000);
            double linear_velocity = velocity_msg->linear.x;
            double angular_velocity = velocity_msg->angular.z;
            double wheel_radius = outdoor_mode_ ? 0.2 : 0.1016;
            double wheel_distance = 0.5;

            double velocity_left_cmd = 0.1 * (linear_velocity - angular_velocity * wheel_distance / 2.0) / wheel_radius;
            double velocity_right_cmd = 0.1 * (linear_velocity + angular_velocity * wheel_distance / 2.0) / wheel_radius;

            left_wheel_motor_.Set(ControlMode::PercentOutput, velocity_left_cmd);
            right_wheel_motor_.Set(ControlMode::PercentOutput, velocity_right_cmd);
        }
    }

    /**
     * @brief Callback for control messages.
     */
    void control_callback(const lunabot_autonomous::msg::Control::SharedPtr control_msg)
    {
        manual_enabled_ = control_msg->enable_manual_drive;

        if (control_msg->enable_intake)
        {
            // TODO: Implement intake mechanism control
        }
        else if (control_msg->enable_outtake)
        {
            // TODO: Implement outtake mechanism control
        }
        else
        {
            auto clock = rclcpp::Clock();
            RCLCPP_INFO_THROTTLE(get_logger(), clock, 1000, "\033[0;33mNO MECHANISM ENABLED\033[0m");
        }
    }

    /**
     * @brief Retrieves button value based on the current controller mode.
     */
    int get_button(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const std::initializer_list<int> &mappings)
    {
        size_t index = xbox_mode_ ? 2 : ps4_mode_ ? 1
                                                  : 0;
        return joy_msg->buttons[*(mappings.begin() + index)];
    }

    /**
     * @brief Retrieves axis value based on the current controller mode.
     */
    double get_axis(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const std::initializer_list<int> &mappings)
    {
        size_t index = xbox_mode_ ? 2 : ps4_mode_ ? 1
                                                  : 0;
        return joy_msg->axes[*(mappings.begin() + index)];
    }

    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber_;
    rclcpp::Subscription<lunabot_autonomous::msg::Control>::SharedPtr control_subscriber_;

    // Control timers
    rclcpp::TimerBase::SharedPtr intake_timer_, outtake_timer_;

    // Robot states, controller modes, and motor speeds
    bool manual_enabled_, robot_disabled_;
    bool xbox_mode_, ps4_mode_, switch_mode_, outdoor_mode_;
    double speed_multiplier_, left_speed_, right_speed_, actuator_speed_, trencher_speed_, door_speed_, vibrator_speed_;

    // Controller states and values
    bool home_button_, share_button_, menu_button_, a_button_, b_button_, x_button_, y_button_;
    double left_trigger_, right_trigger_, left_bumper_, right_bumper_, d_pad_vertical_, d_pad_horizontal_, left_joystick_x_, left_joystick_y_;

    // Motors
    TalonFX left_wheel_motor_{1};
    TalonFX right_wheel_motor_{2};
    TalonSRX actuator_left_motor_{3};
    TalonSRX actuator_right_motor_{4};
    TalonSRX vibrator_motor_{7};
    TalonFX door_motor_{5};
    TalonFX trencher_motor_{6};
};

/**
 * @brief Main function initializes and spins the RobotController node.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}