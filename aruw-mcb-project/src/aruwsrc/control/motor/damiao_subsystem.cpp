#include "damiao_subsystem.hpp"

#include "aruwsrc/control/motor/damiao_motor.hpp"

namespace aruwsrc::control::motor
{
void DamiaoSubsystem::initialize()
{
    motor.initialize();
    motor.enable();
}
void DamiaoSubsystem::refresh()
{
    // Keep the motor's CAN watchdog alive after a command finishes.
    if (!commandSent)
    {
        stop();
    }
    commandSent = false;
}
void DamiaoSubsystem::refreshSafeDisconnect()
{
    stop();
    commandSent = false;
}

void DamiaoSubsystem::setDesiredOutput(int32_t torqueMilliNm)
{
    commandSent = true;
    motor.setDesiredOutput(torqueMilliNm);
}

void DamiaoSubsystem::sendMitCommand(
    float targetPosition,
    float targetVelocity,
    float kp,
    float kd,
    float torqueNm)
{
    commandSent = true;
    motor.sendMitCommand(targetPosition, targetVelocity, kp, kd, torqueNm);
}
void DamiaoSubsystem::sendVeloCommand(float velo)
{
    commandSent = true;
    motor.sendVelocityCommand(velo);
}
}  // namespace aruwsrc::control::motor
