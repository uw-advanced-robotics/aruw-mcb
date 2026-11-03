#ifndef MOTOR_SUBSYSTEM_HPP_
#define MOTOR_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/motor_interface.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/algorithms/smooth_pid.hpp"

using namespace tap::motor;

namespace aruwsrc::motor_tester
{
class MotorSubsystem : public tap::control::Subsystem
{
public:
    MotorSubsystem(tap::Drivers* drivers, MotorInterface& motor, tap::algorithms::SmoothPidConfig pidConfig)
    : Subsystem(drivers),
      motor(motor),
      pidController(pidConfig)
    {}

    // void setDesiredOutput(int32_t output) {
    //     desiredOutput = std::clamp(output, (int32_t) -DjiMotor::MAX_OUTPUT_C620, (int32_t) DjiMotor::MAX_OUTPUT_C620);
    // }

    void setDesiredPosition(float position) {
        desiredPosition = position;
    }

    float getDesiredPosition() {
        return desiredPosition;
    }

    void refresh() override {
        // motor.setDesiredOutput(desiredOutput);
        float error = desiredPosition - motor.getEncoder()->getPosition().getUnwrappedValue();
        float errorDeriv = -motor.getEncoder()->getVelocity();
        float output = pidController.runController(error, errorDeriv, 0.002f);
        motor.setDesiredOutput(std::clamp(output, (float) -DjiMotor::MAX_OUTPUT_C620, (float) DjiMotor::MAX_OUTPUT_C620));
    }

private:
    float desiredPosition;
    tap::motor::MotorInterface& motor;
    tap::algorithms::SmoothPid pidController;
};
}

#endif