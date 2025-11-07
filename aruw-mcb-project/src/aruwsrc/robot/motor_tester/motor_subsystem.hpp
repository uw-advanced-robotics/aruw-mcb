#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"

#ifndef MOTOR_TEST_SUBSYSTEM
#define MOTOR_TEST_SUBSYSTEM
namespace aruwsrc::robot::motor_tester
{
class MotorSubsystem : public tap::control::Subsystem
{
public:
    MotorSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorInterface& motor,
        tap::algorithms::SmoothPidConfig pidConfig)
        : Subsystem(drivers),
          motor(motor),
          pid(pidConfig)
    {
    }

    void initialize() override { motor.initialize(); }

    float getDesiredOutput() { return desiredOutput; }

    void setDesiredOutput(float out) { desiredOutput = out; }

    void setDesiredPosition(float position) { desiredPosition = position; }
    void refresh() override
    {
        refresh2();
        motor.setDesiredOutput(desiredOutput);
    }
    void refreshSafeDisconnect() override { motor.setDesiredOutput(0); }

    void refresh2()
    {
        float currentPos = motor.getEncoder()->getPosition().getUnwrappedValue();
        float errorDerivative = -motor.getEncoder()->getVelocity();
        float output = pid.runController(desiredPosition - currentPos, errorDerivative, 0.002f);
        desiredOutput = output;
    }

private:
    float desiredOutput = 0;
    float desiredPosition = 0.0;
    tap::motor::MotorInterface& motor;
    tap::algorithms::SmoothPid pid;
};
}  // namespace aruwsrc::robot::motor_tester

#endif  // MOTOR_TEST_SUBSYSTEM