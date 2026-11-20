#include "damiao_agitator_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"

namespace motor_tester_control
{
using tap::communication::serial::Remote;

// Debugger watch values for distinguishing a mapping issue from missing CAN feedback.
volatile bool DEBUG_damiao2325LeftSwitchUp = false;
volatile bool DEBUG_damiao2325Online = false;
volatile uint8_t DEBUG_damiao2325Status = 0xFF;
volatile float DEBUG_damiao2325Setpoint = 0.0f;
volatile float DEBUG_damiao2325Velocity = 0.0f;
volatile float DEBUG_damiao2325Position = 0.0f;

DamiaoAgitatorSubsystem::DamiaoAgitatorSubsystem(
    tap::Drivers* drivers,
    aruwsrc::control::motor::DamiaoMotor& motor,
    float jammingVelocityDifference,
    uint32_t jammingTime)
    : tap::control::Subsystem(drivers),
      motor(motor),
      jamChecker(this, jammingVelocityDifference, jammingTime)
{
}

void DamiaoAgitatorSubsystem::initialize()
{
    motor.initialize();
    motor.enable();
}

void DamiaoAgitatorSubsystem::refresh()
{
    DEBUG_damiao2325LeftSwitchUp =
        drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP;
    DEBUG_damiao2325Online = isOnline();
    DEBUG_damiao2325Status = motor.getStatus();
    DEBUG_damiao2325Setpoint = velocitySetpoint;
    DEBUG_damiao2325Velocity = getCurrentValue();
    DEBUG_damiao2325Position = getCurrentValueIntegral();
    if (!isOnline())
    {
        calibrated = false;
        refreshSafeDisconnect();
        return;
    }
    if (!calibrated)
    {
        calibrateHere();
    }
    // Match the driver's configured torque-output cap; zero setpoint gets no feedforward.
    const float maxTorqueNm = aruwsrc::control::motor::DM_S2325_LIMITS.maxOutputMilliNm / 1000.0f;
    const float requestedTorqueNm = torqueFeedforwardNm;
    const float magnitudeNm = requestedTorqueNm > 0.0f
                                  ? tap::algorithms::limitVal(requestedTorqueNm, 0.0f, maxTorqueNm)
                                  : 0.0f;
    const float feedforwardNm = velocitySetpoint > 0.0f ? magnitudeNm
                               : velocitySetpoint < 0.0f ? -magnitudeNm
                                                        : 0.0f;
    motor.sendMitCommand(0.0f, velocitySetpoint, 0.0f, kd, feedforwardNm);
    jammed = jammed || jamChecker.check();
}

void DamiaoAgitatorSubsystem::refreshSafeDisconnect()
{
    velocitySetpoint = 0.0f;
    clearJam();
    motor.stop();
}

float DamiaoAgitatorSubsystem::getCurrentValue() const
{
    return motor.getEncoder()->getVelocity();
}

float DamiaoAgitatorSubsystem::getCurrentValueIntegral() const
{
    return motor.getEncoder()->getPosition().getUnwrappedValue();
}

float DamiaoAgitatorSubsystem::getJamSetpointTolerance() const
{
    return jamChecker.getJamSetpointTolerance();
}

bool DamiaoAgitatorSubsystem::calibrateHere()
{
    if (!isOnline())
    {
        return false;
    }
    motor.getEncoder()->resetEncoderValue();
    calibrated = true;
    clearJam();
    return true;
}

void DamiaoAgitatorSubsystem::clearJam()
{
    jammed = false;
    jamChecker.restart();
}
}  // namespace motor_tester_control
