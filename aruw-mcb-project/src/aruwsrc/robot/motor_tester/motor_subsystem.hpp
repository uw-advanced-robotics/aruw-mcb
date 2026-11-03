#ifndef MOTOR_SUBSYSTEM_HPP_
#define MOTOR_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/motor_interface.hpp"
#include "tap/motor/dji_motor.hpp"

using namespace tap::motor;

namespace aruwsrc::motor_tester
{
class MotorSubsystem : public tap::control::Subsystem
{
public:
    MotorSubsystem(tap::Drivers* drivers, MotorInterface& motor);

    void setDesiredOutput(int32_t output) {
        desiredOutput = std::clamp(output, (int32_t) -DjiMotor::MAX_OUTPUT_C620, (int32_t) DjiMotor::MAX_OUTPUT_C620);
    }

    int32_t getDesiredOutput() {
        return desiredOutput;
    }

    void refresh() override {
        motor.setDesiredOutput(desiredOutput);
    }

private:
    int32_t desiredOutput;
    tap::motor::MotorInterface& motor;
};
}

#endif