#ifndef __TURRET_WORLD_RELATIVE_POSITION_COMMAND_HPP__
#define __TURRET_WORLD_RELATIVE_POSITION_COMMAND_HPP__

#include "src/aruwsrc/algorithms/turret_pid.hpp"
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

    void refresh();

 private:
    static constexpr float YAW_P = 4500.0f;  // 500.0f;
    static constexpr float YAW_I = 0.0f;
    static constexpr float YAW_D = 190.0f;  // 50.0f
    static constexpr float YAW_MAX_ERROR_SUM = 0.0f;
    static constexpr float YAW_MAX_OUTPUT = 32000.0f;  // 16000.0f
    static constexpr float YAW_Q_DERIVATIVE_KALMAN = 1.5f;
    static constexpr float YAW_R_DERIVATIVE_KALMAN = 40.0f;
    static constexpr float YAW_Q_PROPORTIONAL_KALMAN = 1.5f;
    static constexpr float YAW_R_pROPORTIONAL_KALMAN = 11.0f;

    static constexpr float PITCH_P = 5000.0f;
    static constexpr float PITCH_I = 0.0f;
    static constexpr float PITCH_D = 100.0f;
    static constexpr float PITCH_MAX_ERROR_SUM = 0.0f;
    static constexpr float PITCH_MAX_OUTPUT = 32000.0f;
    static constexpr float PITCH_Q_DERIVATIVE_KALMAN = 1.5f;
    static constexpr float PITCH_R_DERIVATIVE_KALMAN = 40.0f;
    static constexpr float PITCH_Q_PROPORTIONAL_KALMAN = 1.5f;
    static constexpr float PITCH_R_PROPORTIONAL_KALMAN = 11.0f;

    TurretSubsystem *turretSubsystem;

    aruwlib::algorithms::ContiguousFloat yawTargetAngle;
    aruwlib::algorithms::ContiguousFloat pitchTargetAngle;

    aruwlib::algorithms::ContiguousFloat currValueImuYawGimbal;
    aruwlib::algorithms::ContiguousFloat currImuPitchAngle;

    float imuInitialYaw;

    float lowPassUserVelocityYaw;
    float lowPassUserVelocityPitch;

    aruwsrc::algorithms::TurretPid yawPid;
    aruwsrc::algorithms::TurretPid pitchPid;

    void runYawPositionController();

    void runPitchPositionController();

    float calcPitchImuOffset();
};

}  // namespace control

}  // namespace aruwsrc

#endif
