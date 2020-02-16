#ifndef __TURRET_CV_COMMAND_H__
#define __TURRET_CV_COMMAND_H__

#include <modm/math/filter/pid.hpp>
#include "src/aruwlib/control/command.hpp"
#include "src/aruwlib/algorithms/contiguous_float.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class TurretSubsystem;
class TurretCVCommand : public Command {
 public:
    explicit TurretCVCommand(TurretSubsystem *subsystem);

    void initialize();
    bool isFinished() const {return false;}

    void execute();

    void end(bool interrupted) { if (interrupted) { return; } }

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

    uint16_t remoteControlScaler = 0.5;

    TurretSubsystem *turretSubsystem;

    aruwlib::algorithms::ContiguousFloat yawTargetAngle;
    aruwlib::algorithms::ContiguousFloat pitchTargetAngle;

    modm::Pid<float> CVYawPid;
    modm::Pid<float> CVPitchPid;

    void updateTurretPosition();
};

}  // namespace control

}  // namespace aruwsrc

#endif
