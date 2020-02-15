#ifndef __TURRET_MANUAL_COMMAND_H__
#define __TURRET_MANUAL_COMMAND_H__

#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/command.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class TurretSubsystem;
class TurretManualCommand : public Command {
 public:
    explicit TurretManualCommand(TurretSubsystem *subsystem);

    void initialize() {}

    void execute();
    void end(bool interrupted) { if (interrupted) { return; } }

    void pitchToVelocity(float degree);
    void yawToVelocity(float degree);

    void pitchIncrementVelocity(float degree);
    void yawIncrementVelocity(float degree);

    float getPitchOutput();
    float getYawOutput();

 private:
    uint16_t YAW_P = 1.0f;
    uint16_t YAW_I = 0.0f;
    uint16_t YAW_D = 0.0f;
    uint16_t YAW_MAX_ERROR_SUM = 0.0f;
    uint16_t YAW_MAX_OUTPUT = 16000;

    uint16_t PITCH_P = 1.0f;
    uint16_t PITCH_I = 0.0f;
    uint16_t PITCH_D = 0.0f;
    uint16_t PITCH_MAX_ERROR_SUM = 0.0f;
    uint16_t PITCH_MAX_OUTPUT = 16000;

    const float remoteControlScaler = 30000;

    TurretSubsystem *turretSubsystem;
    modm::Pid<float> manualYawPid;
    modm::Pid<float> manualPitchPid;

    float yawVelocityTarget = 0;
    float pitchVelocityTarget = 0;

    void updateTurretPosition(void);
};

}  // namespace control

}  // namespace aruwsrc

#endif
