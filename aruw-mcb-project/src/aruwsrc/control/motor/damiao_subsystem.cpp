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
    DEBUG_position = motor.getPosition();
    DEBUG_motorStatus = motor.getStatus();
}
void DamiaoSubsystem::refreshSafeDisconnect()
{
    DEBUG_position = motor.getPosition();
    DEBUG_motorStatus = motor.getStatus();

    stop();
}

void DamiaoSubsystem::sendMitCommand(
    float targetPosition,
    float targetVelocity,
    float kp,
    float kd,
    float torqueNm)
{
    motor.sendMitCommand(targetPosition, targetVelocity, kp, kd, torqueNm);
}
void DamiaoSubsystem::sendVeloCommand(float velo)
{
    ++DEBUG_velocityCommandCount;
    DEBUG_velocityCommandSent = motor.sendVelocityCommand(velo);
}
}  // namespace aruwsrc::control::motor
