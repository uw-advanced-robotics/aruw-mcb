
#ifndef MOTOR_TESTER_SUBSYSTEM_HPP_
#define MOTOR_TESTER_SUBSYSTEM_HPP_

#include <cstdint>

#include "tap/control/subsystem.hpp"
#include "tap/motor/motor_interface.hpp"
#include "tap/algorithms/smooth_pid.hpp"

namespace aruwsrc::motor_tester
{

class MotorTesterSubsystem : public tap::control::Subsystem
{
public:
    MotorTesterSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorInterface& motor,
        const tap::algorithms::SmoothPidConfig& pidConfig)
        : tap::control::Subsystem(drivers),
          motor(motor),
          pid(pidConfig),
          desiredPosition(0.0f)
    {
    }

    void initialize() override
    {
        motor.initialize();
        pid.reset();

        desiredPosition =
            motor.getEncoder()->getPosition().getUnwrappedValue();
    }

    void refresh() override
    {   
        pid.setP(kp);
        pid.setI(ki);
        pid.setD(kd);

        float actualPosition =
            motor.getEncoder()->getPosition().getUnwrappedValue();

        float actualVelocity =
            motor.getEncoder()->getVelocity();

        float error = desiredPosition - actualPosition;
        float errorDerivative = -actualVelocity;

        float output = pid.runController(
            error,
            errorDerivative,
            0.002f
        );

        motor.setDesiredOutput(
            static_cast<int32_t>(output));
    }

    void refreshSafeDisconnect() override
    {
        motor.setDesiredOutput(0);
        pid.reset();

        desiredPosition =
            motor.getEncoder()->getPosition().getUnwrappedValue();
    }

    void setDesiredPosition(float position)
    {
        desiredPosition = position;
    }

    float getDesiredPosition() const
    {
        return desiredPosition;
    }

    const char* getName() const override
    {
        return "Motor Tester";
    }

private:
    tap::motor::MotorInterface& motor;
    tap::algorithms::SmoothPid pid;
    float desiredPosition;

public:
    volatile float kp = 300.0f;
    volatile float ki = 0.0f;
    volatile float kd = 0.0f;
};

}  // namespace aruwsrc::motor_tester

#endif  // MOTOR_TESTER_SUBSYSTEM_HPP_
