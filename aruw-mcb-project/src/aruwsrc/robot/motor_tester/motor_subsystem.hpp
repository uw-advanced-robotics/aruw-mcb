#ifndef MOTOR_SUBSYSTEM_
#define MOTOR_SUBSYSTEM_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/algorithms/wrapped_float.hpp"

namespace aruwsrc::robot::motor_tester {

class MotorSubsystem : public tap::control::Subsystem {
public:
    MotorSubsystem(tap::Drivers* drivers, tap::motor::MotorInterface& motor, tap::algorithms::SmoothPidConfig& pidConfig, float deltaT) :
                    Subsystem(drivers), motor(motor), pid(pidConfig), targetPosition(tap::algorithms::Angle(0)), deltaT(deltaT) {}

    void setTargetVelocity(float targetVelocity_) {
        targetVelocity = targetVelocity_;
    }

    float getTargetPosition() const {
        return targetPosition.getWrappedValue();
    }

    void refresh() override {
        targetPosition += targetVelocity * deltaT;

        tap::algorithms::WrappedFloat error = targetPosition - motor.getEncoder()->getPosition();
        float dError = -motor.getEncoder()->getVelocity();

        float output = pid.runController(error.getUnwrappedValue(), dError, deltaT);

        float cap = tap::motor::DjiMotor::MAX_OUTPUT_C620;
        motor.setDesiredOutput(std::clamp(output, -cap, cap));
    }

private: 
    tap::motor::MotorInterface& motor;
    tap::algorithms::SmoothPid pid;
    tap::algorithms::WrappedFloat targetPosition;
    float targetVelocity;
    float deltaT;
};

} 

#endif // MOTOR_SUBSYSTEM_