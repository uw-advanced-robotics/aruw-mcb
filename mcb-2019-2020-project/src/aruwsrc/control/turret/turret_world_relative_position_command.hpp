#ifndef __TURRET_WORLD_RELATIVE_POSITION_COMMAND_HPP__
#define __TURRET_WORLD_RELATIVE_POSITION_COMMAND_HPP__

#include "src/aruwsrc/algorithms/turret_pid.hpp"
#include "src/aruwlib/control/command.hpp"
#include "src/aruwlib/algorithms/contiguous_float.hpp"
#include "src/aruwsrc/control/turret/turret_subsystem.hpp"
#include "src/aruwsrc/control/chassis/chassis_subsystem.hpp"

namespace aruwsrc
{

namespace control
{

class TurretWorldRelativePositionCommand : public Command
{
 public:
    TurretWorldRelativePositionCommand(TurretSubsystem *subsystem,
                                       chassis::ChassisSubsystem *chassis);

    void initialize();

    bool isFinished() const {return false;}

    void execute();

    void end(bool);

    void refresh();

 private:
    static constexpr float YAW_P = 4500.0f;  // 500.0f;
    static constexpr float YAW_I = 0.0f;
    static constexpr float YAW_D = 140.0f;  // 50.0f
    static constexpr float YAW_MAX_ERROR_SUM = 0.0f;
    static constexpr float YAW_MAX_OUTPUT = 32000.0f;  // 16000.0f
    static constexpr float YAW_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_R_DERIVATIVE_KALMAN = 20.0f;
    static constexpr float YAW_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float YAW_R_PROPORTIONAL_KALMAN = 0.0f;

    static constexpr float PITCH_P = 3500.0f;
    static constexpr float PITCH_I = 0.0f;
    static constexpr float PITCH_D = 80.0f;
    static constexpr float PITCH_MAX_ERROR_SUM = 0.0f;
    static constexpr float PITCH_MAX_OUTPUT = 32000.0f;
    static constexpr float PITCH_Q_DERIVATIVE_KALMAN = 1.5f;
    static constexpr float PITCH_R_DERIVATIVE_KALMAN = 20.0f;
    static constexpr float PITCH_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float PITCH_R_PROPORTIONAL_KALMAN = 2.0f;

    static constexpr float USER_INPUT_LOW_PASS_ALPHA = 0.153f;
    static constexpr float USER_REMOTE_YAW_SCALAR = 0.5f;
    static constexpr float USER_MOUSE_YAW_SCALAR = (1.0f / 1000.0f);
    static constexpr float USER_REMOTE_PITCH_SCALAR = 0.5f;
    static constexpr float USER_MOUSE_PITCH_SCALAR = (1.0f / 1000.0f);

    static constexpr float FEED_FORWARD_KP = 2.75f;
    static constexpr float FEED_FORWARD_SIN_GAIN = 1.0f;
    static constexpr float FEED_FORWARD_KD = 30.0f;
    static constexpr float FEED_FORWARD_MAX_OUTPUT = 20000.0f;
    static constexpr float FEED_FORWARD_DERIVATIVE_LOW_PASS = 0.154f;

    TurretSubsystem *turretSubsystem;
    chassis::ChassisSubsystem *chassisSubsystem;

    aruwlib::algorithms::ContiguousFloat yawTargetAngle;
    aruwlib::algorithms::ContiguousFloat pitchTargetAngle;

    aruwlib::algorithms::ContiguousFloat currValueImuYawGimbal;
    aruwlib::algorithms::ContiguousFloat currImuPitchAngle;

    float imuInitialYaw;

    float lowPassUserVelocityYaw;
    float lowPassUserVelocityPitch;

    float chassisRotationDerivative = 0.0f;
    float prevChassisRotationDesired = 0.0f;

    aruwsrc::algorithms::TurretPid yawPid;
    aruwsrc::algorithms::TurretPid pitchPid;

    void runYawPositionController();

    void runPitchPositionController();

    float calcPitchImuOffset();

    float getUserTurretYawInput();
    float getUserTurretPitchInput();
};

}  // namespace control

}  // namespace aruwsrc

#endif
