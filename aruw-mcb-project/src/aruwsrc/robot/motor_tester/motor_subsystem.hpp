#ifndef MOTOR_SUBSYSTEM_
#define MOTOR_SUBSYSTEM_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"

namespace aruwsrc::robot::motor_tester {

class MotorSubsystem : public tap::control::Subsystem {
public:
    MotorSubsystem(tap::Drivers* drivers, tap::motor::MotorInterface& motor_);

    void setDesiredOutput(int32_t desiredOutput_) {
        int32_t cap = tap::motor::DjiMotor::MAX_OUTPUT_C620;
        desiredOutput = std::clamp(desiredOutput_, -cap, cap);
    }

    int32_t getDesiredOutput() const {
        return desiredOutput;
    }

    void refresh() {
        motor.setDesiredOutput(desiredOutput);
    }

private: 
    tap::motor::MotorInterface& motor;
    int32_t desiredOutput;
};

} 

#endif // MOTOR_SUBSYSTEM_