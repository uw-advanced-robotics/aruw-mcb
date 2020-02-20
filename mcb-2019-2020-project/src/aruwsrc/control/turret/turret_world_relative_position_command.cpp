#include "turret_world_relative_position_command.hpp"
#include "src/aruwlib/communication/sensors/mpu6500/mpu6500.hpp"
#include "src/aruwlib/communication/remote.hpp"  /// \todo fix and remove remote from command
#include "src/aruwlib/algorithms/math_user_utils.hpp"

using namespace aruwlib::sensors;

namespace aruwsrc
{

namespace control
{

TurretWorldRelativePositionCommand::TurretWorldRelativePositionCommand(
    TurretSubsystem *subsystem
) :
    turretSubsystem(subsystem),
    yawTargetAngle(TurretSubsystem::TURRET_START_ANGLE, 0.0f, 360.0f),
    pitchTargetAngle(TurretSubsystem::TURRET_START_ANGLE, 0.0f, 360.0f),
    currValueImuYawGimbal(0.0f, 0.0f, 360.0f),
    currImuPitchAngle(0.0f, 0.0f, 360.0f),
    imuInitialYaw(0.0f),
    lowPassUserVelocityYaw(0.0f),
    lowPassUserVelocityPitch(0.0f),
    yawPid(
        YAW_P,
        YAW_I,
        YAW_D,
        YAW_MAX_ERROR_SUM,
        YAW_MAX_OUTPUT,
        YAW_Q_DERIVATIVE_KALMAN,
        YAW_R_DERIVATIVE_KALMAN,
        YAW_Q_PROPORTIONAL_KALMAN,
        YAW_R_pROPORTIONAL_KALMAN
    ),
    pitchPid(
        PITCH_P,
        PITCH_I,
        PITCH_D,
        PITCH_MAX_ERROR_SUM,
        PITCH_MAX_OUTPUT,
        PITCH_Q_DERIVATIVE_KALMAN,
        PITCH_R_DERIVATIVE_KALMAN,
        PITCH_Q_PROPORTIONAL_KALMAN,
        PITCH_R_PROPORTIONAL_KALMAN
    )
{
    addSubsystemRequirement(dynamic_cast<Subsystem*>(subsystem));
}

void TurretWorldRelativePositionCommand::initialize()
{
    imuInitialYaw = Mpu6500::getImuAttitude().yaw;
}

void TurretWorldRelativePositionCommand::execute()
{
    runYawPositionController();
    runPitchPositionController();
}

/// \todo add min/max limiting to the turret here

aruwlib::algorithms::ContiguousFloat min(0.0f, 0.0f, 360.0f);
aruwlib::algorithms::ContiguousFloat max(0.0f, 0.0f, 360.0f);
bool outOfBounds = false;

void TurretWorldRelativePositionCommand::runYawPositionController()
{
    /// \todo fix user input, not my problem
    float userVelocity = static_cast<float>(aruwlib::Remote::getChannel(aruwlib::Remote::Channel::RIGHT_HORIZONTAL)) * 0.5f
        - static_cast<float>(aruwlib::Remote::getMouseX()) / 1000.0f;
    lowPassUserVelocityYaw = 0.13f * userVelocity + (1 - 0.13f) * lowPassUserVelocityYaw;

    // calculate the desired user angle in world reference frame
    // if user does not want to move the turret, recalibrate the imu initial value
    yawTargetAngle.shiftValue(-lowPassUserVelocityYaw);
    // the position controller is in world reference frame (i.e. add imu yaw to current encoder value)
    currValueImuYawGimbal.setValue(turretSubsystem->getYawAngle().getValue()
            + Mpu6500::getImuAttitude().yaw - imuInitialYaw);


    // limit currValueImuYawGimbal
    /// \todo test this
    /// \todo replace with constants
    min.setValue(0.0f + Mpu6500::getImuAttitude().yaw);
    max.setValue(180.0f + Mpu6500::getImuAttitude().yaw);

    outOfBounds = false;
    if (min.getValue() < max.getValue())
    {
        if (yawTargetAngle.getValue() > max.getValue() || yawTargetAngle.getValue() < min.getValue())
        {
            outOfBounds = true;
        }
    }
    else
    {
        if (yawTargetAngle.getValue() > max.getValue() && yawTargetAngle.getValue() < min.getValue())
        {
            outOfBounds = true;
        }
    }
    if (outOfBounds)
    {
        float targetMinDifference = fabs(yawTargetAngle.difference(min));
        float targetMaxDifference = fabs(yawTargetAngle.difference(max));
        yawTargetAngle.setValue(targetMinDifference < targetMaxDifference ? min.getValue() : max.getValue());
    }

    float positionControllerError = currValueImuYawGimbal.difference(yawTargetAngle);
    float pidOutput = yawPid.runController(positionControllerError,
        turretSubsystem->getYawVelocity() + Mpu6500::getGz());  /// \todo fix gz in mpu6500 class

    turretSubsystem->setYawMotorOutput(pidOutput);
}

void TurretWorldRelativePositionCommand::runPitchPositionController()
{
    /// \todo fix user input
    float userVelocity = static_cast<float>(aruwlib::Remote::getChannel(aruwlib::Remote::Channel::RIGHT_VERTICAL)) * 0.5f
        - static_cast<float>(aruwlib::Remote::getMouseX()) / 1000.0f;
    lowPassUserVelocityPitch = 0.13f * userVelocity + (1 - 0.13f) * lowPassUserVelocityPitch;

    // calculate the desired user angle in world reference frame
    // if user does not want to move the turret, recalibrate the imu initial value
    pitchTargetAngle.shiftValue(lowPassUserVelocityPitch);
    // the position controller is in world reference frame (i.e. add imu yaw to current encoder value)
    currImuPitchAngle.setValue(turretSubsystem->getPitchAngle().getValue());

    float positionControllerError = currImuPitchAngle.difference(pitchTargetAngle);
    float pidOutput = pitchPid.runController(positionControllerError,
        turretSubsystem->getPitchAngle().getValue());  /// \todo fix imu stuff for pitch

    turretSubsystem->setPitchMotorOutput(pidOutput);
}

}  // namespace control

}  // namespace aruwsrc
