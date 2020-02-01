#ifndef __TURRET_CV_COMMAND_H__
#define __TURRET_CV_COMMAND_H__

#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/command.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class TurretSubsystem;
class TurretCVCommand : public Command {
 public:
    explicit TurretCVCommand(TurretSubsystem *turret = nullptr);

    void initialize(void);

    void execute(void);

    void end(bool interrupted);

    bool isFinished(void) const;

    void pitchToEncoder(float degree);
    void yawToEncoder(float degree);

    void pitchIncrementEncoder(float degree);
    void yawIncrementEncoder(float degree);

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

    const float remoteControlScaler = 0.5;

    modm::Pid<float>::Parameter *CVYawPid;
    modm::Pid<float>::Parameter *CVPitchPid;

    TurretSubsystem *turretSubsystem;

    float pitchEncoderTarget;
    float yawEncoderTarget;

    void updateTurretPosition(void);
};

}  // control

}  // aruwsrc

#endif
