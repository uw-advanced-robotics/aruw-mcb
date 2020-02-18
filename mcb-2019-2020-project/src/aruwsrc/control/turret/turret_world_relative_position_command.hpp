#ifndef __TURRET_WORLD_RELATIVE_POSITION_COMMAND_HPP__
#define __TURRET_WORLD_RELATIVE_POSITION_COMMAND_HPP__

#include "src/aruwlib/algorithms/"
#include "src/aruwlib/control/command.hpp"
#include "src/aruwlib/algorithms/contiguous_float.hpp"
#include "src/aruwsrc/control/turret/turret_subsystem.hpp"

namespace aruwsrc
{

namespace control
{

class TurretWorldRelativePositionCommand : public Command
{
 public:
    explicit TurretWorldRelativePositionCommand(TurretSubsystem *subsystem);

    void initialize();

    bool isFinished() const {return false;}

    void execute();

    void end(bool interrupted) { if (interrupted) { return; } }

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

    TurretSubsystem *turretSubsystem;

    aruwlib::algorithms::ContiguousFloat yawTargetAngle;
    aruwlib::algorithms::ContiguousFloat pitchTargetAngle;

    modm::Pid<float> CVYawPid;
    modm::Pid<float> CVPitchPid;

    void updateTurretPosition();

    void pitchIncrementAngle(float angle);
    void yawIncrementAngle(float angle);
};

}  // namespace control

}  // namespace aruwsrc

#endif
