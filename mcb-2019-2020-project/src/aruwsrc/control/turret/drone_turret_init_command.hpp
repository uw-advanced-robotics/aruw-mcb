#ifndef __DRONE_TURRET_INIT_COMMAND_H__
#define __DRONE_TURRET_INIT_COMMAND_H__

#include <modm/math/filter/pid.hpp>
#include <aruwlib/control/command.hpp>
#include <aruwlib/algorithms/contiguous_float.hpp>

using namespace aruwlib::control;

namespace aruwsrc
{

namespace turret
{

class DroneTurretSubsystem;
class DroneTurretInitCommand : public Command {
 public:
    explicit DroneTurretInitCommand(DroneTurretSubsystem *subsystem);

    void initialize() {}
    bool isFinished() const;

    void execute();
    void end(bool);

 private:
    const float YAW_P = 300.0f;
    const float YAW_I = 0.0f;
    const float YAW_D = 100.0f;
    const float YAW_MAX_ERROR_SUM = 0.0f;
    const float YAW_MAX_OUTPUT = 16000;

    const float PITCH_P = 300.0f;
    const float PITCH_I = 0.0f;
    const float PITCH_D = 100.0f;
    const float PITCH_MAX_ERROR_SUM = 0.0f;
    const float PITCH_MAX_OUTPUT = 16000;

    const float pitchTargetAngle = 90.0f;
    const float yawTargetAngle = 90.0f;

    static constexpr float PITCH_GRAVITY_COMPENSATION_KP = 2500.0f;

    DroneTurretSubsystem *turretSubsystem;

    modm::Pid<float> initYawPid;
    modm::Pid<float> initPitchPid;

    void updateTurretPosition(void);
};

}  // namespace DroneTurret

}  // namespace aruwsrc

#endif
