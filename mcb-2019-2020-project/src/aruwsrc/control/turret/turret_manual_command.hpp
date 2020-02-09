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
    explicit TurretManualCommand(TurretSubsystem *turret = nullptr);

    void initialize(void);

    void execute(void);

    void end(bool interrupted);

    bool isFinished(void) const;

    void pitchToVelocity(float degree);
    void yawToVelocity(float degree);

    void pitchIncrementVelocity(float degree);
    void yawIncrementVelocity(float degree);

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

    const float remoteControlScaler = 3;

    modm::Pid<float>::Parameter *manualYawPid;
    modm::Pid<float>::Parameter *manualPitchPid;

    TurretSubsystem *turretSubsystem;

    float pitchVelocityTarget;
    float yawVelocityTarget;

    void updateTurretPosition(void);
};

}  // control

}  // aruwsrc

#endif
