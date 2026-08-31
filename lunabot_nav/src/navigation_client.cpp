/**
 * @file navigation_client.cpp
 * @author Grayson Arendt
 * @date 02/22/2026
 */

#include "lunabot_logger/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "lunabot_msgs/action/depositing.hpp"
#include "lunabot_msgs/action/excavation.hpp"
#include "lunabot_msgs/msg/motor_commands.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include <chrono>
#include <thread>

enum class CompetitionMode { KSC, UCF };

static constexpr double drive_speed = 0.15;
static constexpr int forward_drive_seconds = 25;
static constexpr double rotation_speed = 0.15;
static constexpr int rotation_90_deg_seconds = 10;

static constexpr double first_intermediate_waypoint_x = 0.0;
static constexpr double first_intermediate_waypoint_y = 3.0;

static constexpr double intermediate_waypoint_x = 4.8;
static constexpr double intermediate_waypoint_y = 3.0;

static constexpr double ksc_construction_zone_x = 4.5;
static constexpr double ksc_construction_zone_y = 0.4;

static constexpr double ucf_construction_zone_x = 6.1;
static constexpr double ucf_construction_zone_y = -3.0;

/**
 * @class NavigationClient
 * @brief Runs autonomous one cycle sequence: navigation, excavation, and depositing actions.
 */
class NavigationClient : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using Excavation = lunabot_msgs::action::Excavation;
  using GoalHandleExcavation = rclcpp_action::ClientGoalHandle<Excavation>;
  using Depositing = lunabot_msgs::action::Depositing;
  using GoalHandleDepositing = rclcpp_action::ClientGoalHandle<Depositing>;

  enum class State {
    IDLE,
    DRIVING_FROM_START,
    EXCAVATING,
    ROTATING,
    NAVIGATING_TO_FIRST_INTERMEDIATE,
    ROTATING_AFTER_FIRST_INTERMEDIATE,
    NAVIGATING_TO_INTERMEDIATE,
    NAVIGATING_TO_CONSTRUCTION,
    DEPOSITING,
    COMPLETE
  };

  /**
   * @brief Constructor for the NavigationClient class.
   */
  NavigationClient() : Node("navigation_client")
  {
    this->declare_parameter("mode", "ucf");
    std::string mode_str = this->get_parameter("mode").as_string();

    if (mode_str == "ksc")
    {
      mode_ = CompetitionMode::UCF;
      current_state_ = State::EXCAVATING;
      LOGGER_INFO(this->get_logger(), "UCF Mode");
    } else
    {
      mode_ = CompetitionMode::KSC;
      current_state_ = State::NAVIGATING_TO_FIRST_INTERMEDIATE;
      LOGGER_INFO(this->get_logger(), "KSC Mode");
    }

    navigation_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    excavation_client_ = rclcpp_action::create_client<Excavation>(this, "auto_excavation_action");
    depositing_client_ = rclcpp_action::create_client<Depositing>(this, "auto_depositing_action");
    motor_cmd_publisher_ =
      this->create_publisher<lunabot_msgs::msg::MotorCommands>("/motor_commands", 10);

    execution_timer_ =
      this->create_wall_timer(std::chrono::seconds(1), std::bind(&NavigationClient::execute, this));
  }

private:
  /**
   * @brief Runs the main execution sequence based on current state.
   */
  void execute()
  {
    switch (current_state_)
    {
      case State::IDLE:
        break;
      case State::DRIVING_FROM_START:
        drive_forward_from_start();
        current_state_ = State::EXCAVATING;
        break;
      case State::EXCAVATING:
        request_excavation();
        current_state_ = State::IDLE;
        break;
      case State::ROTATING:
        rotate_90_degrees();
        if (mode_ == CompetitionMode::UCF)
        {
          current_state_ = State::NAVIGATING_TO_CONSTRUCTION;
        } else
        {
          current_state_ = State::NAVIGATING_TO_INTERMEDIATE;
        }
        break;
      case State::NAVIGATING_TO_FIRST_INTERMEDIATE:
        request_navigation_to_first_intermediate();
        current_state_ = State::IDLE;
        break;
      case State::ROTATING_AFTER_FIRST_INTERMEDIATE:
        rotate_90_degrees();
        current_state_ = State::NAVIGATING_TO_INTERMEDIATE;
        break;
      case State::NAVIGATING_TO_INTERMEDIATE:
        request_navigation_to_intermediate();
        current_state_ = State::IDLE;
        break;
      case State::NAVIGATING_TO_CONSTRUCTION:
        request_navigation_to_construction();
        current_state_ = State::IDLE;
        break;
      case State::DEPOSITING:
        request_depositing();
        current_state_ = State::IDLE;
        break;
      case State::COMPLETE:
        LOGGER_SUCCESS(this->get_logger(), "Cycle complete!");
        execution_timer_->cancel();
        break;
    }
  }

  /**
   * @brief Drives robot forward from starting position.
   */
  void drive_forward_from_start()
  {
    LOGGER_ACTION(
      this->get_logger(), "Driving forward from start for %d seconds...", forward_drive_seconds);

    auto motor_cmd = lunabot_msgs::msg::MotorCommands();
    motor_cmd.left_actuator = 0.0;
    motor_cmd.right_actuator = 0.0;
    motor_cmd.vibration = 0.0;

    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time <
           std::chrono::seconds(forward_drive_seconds))
    {
      motor_cmd.left_wheel = drive_speed;
      motor_cmd.right_wheel = -drive_speed;
      motor_cmd_publisher_->publish(motor_cmd);

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    motor_cmd.left_wheel = 0.0;
    motor_cmd.right_wheel = 0.0;
    motor_cmd_publisher_->publish(motor_cmd);

    LOGGER_SUCCESS(this->get_logger(), "Forward drive complete. Ready for excavation.");
  }

  void rotate_90_degrees()
  {
    LOGGER_ACTION(
      this->get_logger(), "Rotating 90 degrees to the right for %d seconds...",
      rotation_90_deg_seconds);

    auto motor_cmd = lunabot_msgs::msg::MotorCommands();
    motor_cmd.left_actuator = 0.0;
    motor_cmd.right_actuator = 0.0;
    motor_cmd.vibration = 0.0;

    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time <
           std::chrono::seconds(rotation_90_deg_seconds))
    {
      if (mode_ == CompetitionMode::UCF)
      {
        motor_cmd.left_wheel = rotation_speed;
        motor_cmd.right_wheel = rotation_speed;
      } else
      {
        motor_cmd.left_wheel = -rotation_speed;
        motor_cmd.right_wheel = -rotation_speed;
      }
      motor_cmd_publisher_->publish(motor_cmd);

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    motor_cmd.left_wheel = 0.0;
    motor_cmd.right_wheel = 0.0;
    motor_cmd_publisher_->publish(motor_cmd);

    LOGGER_SUCCESS(this->get_logger(), "Rotation complete. Ready for navigation.");
  }

  /**
   * @brief Sends a request to the excavation server.
   */
  void request_excavation()
  {
    if (!excavation_client_->wait_for_action_server(std::chrono::seconds(30)))
    {
      LOGGER_WARN_ONCE(this->get_logger(), "Excavation action server not available.");
      return;
    }

    auto goal_msg = Excavation::Goal();
    auto send_goal_options = rclcpp_action::Client<Excavation>::SendGoalOptions();
    send_goal_options.result_callback =
      std::bind(&NavigationClient::handle_excavation_result, this, std::placeholders::_1);

    excavation_client_->async_send_goal(goal_msg, send_goal_options);
  }

  /**
   * @brief Callback for the result of the excavation action.
   * @param result The result of the excavation action.
   */
  void handle_excavation_result(const GoalHandleExcavation::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      LOGGER_SUCCESS(this->get_logger(), "Excavation success! Rotating 90 degrees...");
      current_state_ = State::ROTATING;
    } else
    {
      LOGGER_FAILURE(this->get_logger(), "Excavation failed.");
      current_state_ = State::IDLE;
    }
  }

  void request_navigation_to_first_intermediate()
  {
    LOGGER_INFO(this->get_logger(), "Waiting for Nav2 action server to be ready...");
    if (!navigation_client_->wait_for_action_server(std::chrono::seconds(30)))
    {
      LOGGER_FAILURE(this->get_logger(), "Navigation action server not available after 30s.");
      return;
    }

    LOGGER_INFO(this->get_logger(), "Nav2 action server ready!");

    auto goal_msg = NavigateToPose::Goal();
    geometry_msgs::msg::Pose goal_pose;

    goal_pose.position.x = first_intermediate_waypoint_x;
    goal_pose.position.y = first_intermediate_waypoint_y;
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.z = 0.0;
    goal_pose.orientation.w = 1.0;

    goal_msg.pose.pose = goal_pose;
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.header.frame_id = "map";

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(
      &NavigationClient::handle_first_intermediate_navigation_result, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(
      &NavigationClient::handle_first_intermediate_feedback, this, std::placeholders::_1,
      std::placeholders::_2);

    LOGGER_ACTION(
      this->get_logger(), "Sending navigation goal to first intermediate waypoint [%.1f, %.1f]...",
      first_intermediate_waypoint_x, first_intermediate_waypoint_y);
    navigation_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void handle_first_intermediate_feedback(
    GoalHandleNavigate::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    LOGGER_INFO(
      this->get_logger(), "Distance to first intermediate waypoint: %.2f meters",
      feedback->distance_remaining);
  }

  void handle_first_intermediate_navigation_result(const GoalHandleNavigate::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      LOGGER_SUCCESS(
        this->get_logger(), "First intermediate waypoint reached. Rotating 90 degrees...");
      current_state_ = State::ROTATING_AFTER_FIRST_INTERMEDIATE;
    } else if (result.code == rclcpp_action::ResultCode::ABORTED)
    {
      LOGGER_WARN(this->get_logger(), "First intermediate waypoint aborted, rotating anyway...");
      current_state_ = State::ROTATING_AFTER_FIRST_INTERMEDIATE;
    } else if (result.code == rclcpp_action::ResultCode::CANCELED)
    {
      LOGGER_WARN(this->get_logger(), "First intermediate waypoint canceled, rotating anyway...");
      current_state_ = State::ROTATING_AFTER_FIRST_INTERMEDIATE;
    } else
    {
      LOGGER_WARN(this->get_logger(), "First intermediate waypoint failed, rotating anyway...");
      current_state_ = State::ROTATING_AFTER_FIRST_INTERMEDIATE;
    }
  }

  void request_navigation_to_intermediate()
  {
    LOGGER_INFO(this->get_logger(), "Waiting for Nav2 action server to be ready...");
    if (!navigation_client_->wait_for_action_server(std::chrono::seconds(30)))
    {
      LOGGER_FAILURE(this->get_logger(), "Navigation action server not available after 30s.");
      return;
    }

    LOGGER_INFO(this->get_logger(), "Nav2 action server ready!");

    auto goal_msg = NavigateToPose::Goal();
    geometry_msgs::msg::Pose goal_pose;

    goal_pose.position.x = intermediate_waypoint_x;
    goal_pose.position.y = intermediate_waypoint_y;
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.z = 0.7071068;
    goal_pose.orientation.w = 0.7071068;

    goal_msg.pose.pose = goal_pose;
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.header.frame_id = "map";

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(
      &NavigationClient::handle_intermediate_navigation_result, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(
      &NavigationClient::handle_intermediate_feedback, this, std::placeholders::_1,
      std::placeholders::_2);

    LOGGER_ACTION(
      this->get_logger(), "Sending navigation goal to intermediate waypoint [%.1f, %.1f]...",
      intermediate_waypoint_x, intermediate_waypoint_y);
    navigation_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void handle_intermediate_feedback(
    GoalHandleNavigate::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    LOGGER_INFO(
      this->get_logger(), "Distance to intermediate waypoint: %.2f meters",
      feedback->distance_remaining);
  }

  void handle_intermediate_navigation_result(const GoalHandleNavigate::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      LOGGER_SUCCESS(this->get_logger(), "Intermediate waypoint reached. Navigating to berm...");
      current_state_ = State::NAVIGATING_TO_CONSTRUCTION;
    } else if (result.code == rclcpp_action::ResultCode::ABORTED)
    {
      LOGGER_WARN(
        this->get_logger(), "Intermediate waypoint aborted, proceeding to berm anyway...");
      current_state_ = State::NAVIGATING_TO_CONSTRUCTION;
    } else if (result.code == rclcpp_action::ResultCode::CANCELED)
    {
      LOGGER_WARN(
        this->get_logger(), "Intermediate waypoint canceled, proceeding to berm anyway...");
      current_state_ = State::NAVIGATING_TO_CONSTRUCTION;
    } else
    {
      LOGGER_WARN(this->get_logger(), "Intermediate waypoint failed, proceeding to berm anyway...");
      current_state_ = State::NAVIGATING_TO_CONSTRUCTION;
    }
  }

  /**
   * @brief Sends goal request to navigation server for construction zone.
   */
  void request_navigation_to_construction()
  {
    LOGGER_INFO(this->get_logger(), "Waiting for Nav2 action server to be ready...");
    if (!navigation_client_->wait_for_action_server(std::chrono::seconds(30)))
    {
      LOGGER_FAILURE(this->get_logger(), "Navigation action server not available after 30s.");
      return;
    }

    LOGGER_INFO(this->get_logger(), "Nav2 action server ready!");

    auto goal_msg = NavigateToPose::Goal();
    geometry_msgs::msg::Pose goal_pose;

    double construction_zone_x, construction_zone_y;
    if (mode_ == CompetitionMode::UCF)
    {
      construction_zone_x = ucf_construction_zone_x;
      construction_zone_y = ucf_construction_zone_y;
    } else
    {
      construction_zone_x = ksc_construction_zone_x;
      construction_zone_y = ksc_construction_zone_y;
    }

    goal_pose.position.x = construction_zone_x;
    goal_pose.position.y = construction_zone_y;
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.z = 1.0;
    goal_pose.orientation.w = 0.0;

    goal_msg.pose.pose = goal_pose;
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.header.frame_id = "map";

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(
      &NavigationClient::handle_construction_navigation_result, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(
      &NavigationClient::handle_construction_feedback, this, std::placeholders::_1,
      std::placeholders::_2);

    LOGGER_ACTION(
      this->get_logger(), "Sending navigation goal to berm zone [%.1f, %.1f]...",
      construction_zone_x, construction_zone_y);
    navigation_client_->async_send_goal(goal_msg, send_goal_options);
  }

  /**
   * @brief Callback for navigation feedback to construction zone.
   * @param goal_handle The goal handle.
   * @param feedback The feedback message containing distance remaining.
   */
  void handle_construction_feedback(
    GoalHandleNavigate::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    LOGGER_INFO(
      this->get_logger(), "Distance to berm zone: %.2f meters", feedback->distance_remaining);
  }

  /**
   * @brief Callback for the result of navigation to construction zone.
   * @param result The result of the goal execution.
   */
  void handle_construction_navigation_result(const GoalHandleNavigate::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      LOGGER_SUCCESS(this->get_logger(), "Berm zone reached. Requesting deposit...");
      current_state_ = State::DEPOSITING;
    } else if (result.code == rclcpp_action::ResultCode::ABORTED)
    {
      LOGGER_FAILURE(this->get_logger(), "Goal aborted, unable to reach berm zone");
      current_state_ = State::IDLE;
    } else if (result.code == rclcpp_action::ResultCode::CANCELED)
    {
      LOGGER_WARN(this->get_logger(), "Navigation to berm zone canceled");
      current_state_ = State::IDLE;
    } else
    {
      LOGGER_FAILURE(this->get_logger(), "Navigation to berm zone failed with unknown result");
      current_state_ = State::IDLE;
    }
  }

  /**
   * @brief Sends a request to the depositing server.
   */
  void request_depositing()
  {
    if (!depositing_client_->wait_for_action_server(std::chrono::seconds(30)))
    {
      LOGGER_WARN_ONCE(this->get_logger(), "Depositing action server not available.");
      return;
    }

    auto goal_msg = Depositing::Goal();
    auto send_goal_options = rclcpp_action::Client<Depositing>::SendGoalOptions();
    send_goal_options.result_callback =
      std::bind(&NavigationClient::handle_depositing_result, this, std::placeholders::_1);

    depositing_client_->async_send_goal(goal_msg, send_goal_options);
  }

  /**
   * @brief Callback for the result of the depositing action.
   * @param result The result of the depositing action.
   */
  void handle_depositing_result(const GoalHandleDepositing::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      LOGGER_SUCCESS(this->get_logger(), "Depositing success! Cycle complete!");
      current_state_ = State::COMPLETE;
    } else
    {
      LOGGER_FAILURE(this->get_logger(), "Depositing failed.");
      current_state_ = State::IDLE;
    }
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
  rclcpp_action::Client<Excavation>::SharedPtr excavation_client_;
  rclcpp_action::Client<Depositing>::SharedPtr depositing_client_;
  rclcpp::Publisher<lunabot_msgs::msg::MotorCommands>::SharedPtr motor_cmd_publisher_;
  rclcpp::TimerBase::SharedPtr execution_timer_;

  State current_state_;
  CompetitionMode mode_;
};

/**
 * @brief Main function.
 * Initializes and runs the NavigationClient node.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationClient>());
  rclcpp::shutdown();
  return 0;
}
