#ifndef __TURRET_MANUAL_COMMAND_H__
#define __TURRET_MANUAL_COMMAND_H__

#include <modm/math/filter/pid.hpp>
#include <aruwlib/control/command.hpp>

using namespace aruwlib::control;

namespace aruwsrc
{

namespace turret
{

class DroneTurretSubsystem;
class DroneTurretManualCommand : public Command {
 public:
    explicit DroneTurretManualCommand(DroneTurretSubsystem *subsystem);

    void initialize() {}
    bool isFinished() const;

    void execute();
    void end(bool);

 private:
    const float USER_INPUT_SCALAR = 200.0f;

    const float YAW_P = 20.0f;
    const float YAW_I = 0.0f;
    const float YAW_D = 0.0f;
    const float YAW_MAX_ERROR_SUM = 0.0f;
    const float YAW_MAX_OUTPUT = 16000;

    const float PITCH_P = 20.0f;
    const float PITCH_I = 0.0f;
    const float PITCH_D = 0.0f;
    const float PITCH_MAX_ERROR_SUM = 0.0f;
    const float PITCH_MAX_OUTPUT = 16000;

    static constexpr float PITCH_GRAVITY_COMPENSATION_KP = 2700.0f;

    DroneTurretSubsystem *turretSubsystem;
    modm::Pid<float> manualYawPid;
    modm::Pid<float> manualPitchPid;

    float yawVelocityTarget = 0;
    float pitchVelocityTarget = 0;

    void updateTurretVelocity();

    float getRemoteXMovement() const;
    float getRemoteYMovement() const;

    float getMouseXMovement() const;
    float getMouseYMovement() const;
};

}  // namespace turret

}  // namespace aruwsrc

#endif
