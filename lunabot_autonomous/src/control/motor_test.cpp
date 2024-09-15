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

class MotorTest : public rclcpp::Node {
 public:
  MotorTest() : Node("motor_test"), current(0) {
    controller_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&MotorTest::callbackMotors, this, std::placeholders::_1));

    /* Factory Default all hardware to prevent unexpected behaviour */
    magnet.ConfigFactoryDefault();

		/* setup a basic closed loop */
		magnet.SetNeutralMode(NeutralMode::Brake);

    /* set the peak and nominal outputs, 12V means full */
    magnet.ConfigNominalOutputForward(0, 30);
    magnet.ConfigNominalOutputReverse(0, 30);
    magnet.ConfigPeakOutputForward(1, 30);
    magnet.ConfigPeakOutputReverse(-1, 30);

    /* set closed loop gains in slot0 */
    magnet.Config_kF(0, 0.0, 30);
    magnet.Config_kP(0, 0.1, 30);
    magnet.Config_kI(0, 0.001, 30);
    magnet.Config_kD(0, 0.0, 30);
  }

 private:
  void callbackMotors(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    ctre::phoenix::unmanaged::Unmanaged::FeedEnable(1000);

    d_pad_vertical_ = joy_msg->axes[5];

    magnet.Set(ControlMode::Current, current);

    if (d_pad_vertical_ == 1.0)  // dpad_up
    {
      current += 0.05;
    } else if (d_pad_vertical_ == -1.0)  // dpad_down
    {
      current -= 0.05;
    }

    RCLCPP_INFO(get_logger(), "\033[0;35mMEASURED CURRENT: %f SET CURRENT: %f\033[0m", magnet.GetOutputCurrent(), current);
  }

  double d_pad_vertical_;
  double current = 0.0;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controller_subscriber;

  TalonSRX magnet{7};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotorTest>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
