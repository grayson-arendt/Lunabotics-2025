#include <iostream>
#include <chrono>
#include <iomanip> 
#include <limits> 

#define Phoenix_No_WPI
#include "ctre/Phoenix.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

TalonFX left_wheel_motor_{1};
TalonFX right_wheel_motor_{2};
TalonSRX actuator_left_motor_{3};
TalonSRX actuator_right_motor_{4};
TalonFX trencher_motor_{6};
double left_encoder = 0.0;
double right_encoder = 0.0;
double wheel_diameter = 0.2921;

double meters_to_encoder(double distance_in_meters) {
    return (distance_in_meters * 91500.0) / (wheel_diameter * 3.14159);
}
double encoder_to_meters(double encoder_ticks) {
    return (encoder_ticks * (3.14159 * wheel_diameter)) / 91500.0;
}

int main(int argc, char **argv) 
{
    left_wheel_motor_.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
    right_wheel_motor_.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
 
    left_wheel_motor_.SetSelectedSensorPosition(0, 0, 10);
    right_wheel_motor_.SetSelectedSensorPosition(0, 0, 10);

    right_wheel_motor_.SetInverted(true);

    std::cout << "TRENCHER DOWN" << std::endl;

    for (auto start = std::chrono::steady_clock::now(), now = start; now < start + std::chrono::seconds{13}; now = std::chrono::steady_clock::now()) {
        ctre::phoenix::unmanaged::Unmanaged::FeedEnable(1000);

        trencher_motor_.Set(ControlMode::PercentOutput, -0.6);

        actuator_left_motor_.Set(ControlMode::PercentOutput, 0.3);
        actuator_right_motor_.Set(ControlMode::PercentOutput, 0.3);
    }

    std::cout << "DRIVING FORWARD..." << std::endl;

    actuator_left_motor_.Set(ControlMode::PercentOutput, 0.0);
    actuator_right_motor_.Set(ControlMode::PercentOutput, 0.0);

    while (left_encoder >= -meters_to_encoder(2.0) && right_encoder >= -meters_to_encoder(2.0)) {
        ctre::phoenix::unmanaged::Unmanaged::FeedEnable(1000);

        left_encoder = left_wheel_motor_.GetSelectedSensorPosition();
        right_encoder = right_wheel_motor_.GetSelectedSensorPosition();

        std::cout << std::setprecision(15) << "LEFT ENCODER: " << left_encoder << std::endl; 
        std::cout << std::setprecision(15) << "RIGHT ENCODER: " << right_encoder << std::endl; 
        std::cout << std::setprecision(5) << "DISTANCE DRIVEN: " << -encoder_to_meters(right_encoder) << std::endl; 
  
        left_wheel_motor_.Set(ControlMode::PercentOutput, -0.2);
        right_wheel_motor_.Set(ControlMode::PercentOutput, -0.2);
        trencher_motor_.Set(ControlMode::PercentOutput, -0.6);
    }

    left_wheel_motor_.Set(ControlMode::PercentOutput, 0.0);
    right_wheel_motor_.Set(ControlMode::PercentOutput, 0.0);

    std::cout << "TRENCHER UP" << std::endl;

    for (auto start = std::chrono::steady_clock::now(), now = start; now < start + std::chrono::seconds{18}; now = std::chrono::steady_clock::now()) {
        ctre::phoenix::unmanaged::Unmanaged::FeedEnable(1000);

        trencher_motor_.Set(ControlMode::PercentOutput, -0.6);

        actuator_left_motor_.Set(ControlMode::PercentOutput, -0.3);
        actuator_right_motor_.Set(ControlMode::PercentOutput, -0.3);
    }

    std::cout << "DIGGING COMPLETE" << std::endl;

    trencher_motor_.Set(ControlMode::PercentOutput, 0.0);
    actuator_left_motor_.Set(ControlMode::PercentOutput, 0.0);
    actuator_right_motor_.Set(ControlMode::PercentOutput, 0.0);

  return 0;
}
