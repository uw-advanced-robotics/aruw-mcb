#ifndef DAMIAO_SUBSYSTEM_HPP_
#define DAMIAO_SUBSYSTEM_HPP_
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/motor/damiao_motor.hpp"

namespace aruwsrc::control::motor
{
class DamiaoSubsystem : public tap::control::Subsystem
{
    tap::Drivers* drivers;
    DamiaoMotor& motor;
    float DEBUG_position = 0.0f;
    uint8_t DEBUG_motorStatus = 0xFF;
    uint32_t DEBUG_velocityCommandCount = 0;
    bool DEBUG_velocityCommandSent = false;

public:
    DamiaoSubsystem(tap::Drivers* drivers, DamiaoMotor& motor) : Subsystem(drivers), motor(motor) {}

    void initialize() override;

    void refresh() override;

    void refreshSafeDisconnect() override;
    void sendMitCommand(float targetPosition, float targetVelocity, float kp, float kd, float torqueNm);
    void sendVeloCommand(float velo);
    void stop() { motor.stop(); }
    
};

}  // namespace aruwsrc::control::motor

#endif  // DAMIAO_SUBSYSTEM_HPP_
