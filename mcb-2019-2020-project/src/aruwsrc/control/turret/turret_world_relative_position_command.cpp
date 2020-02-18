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
        PITCH_R_pROPORTIONAL_KALMAN
    )
{}

void TurretWorldRelativePositionCommand::initialize()
{
    imuInitialYaw = Mpu6500::getImuAttitude().yaw;
}

void TurretWorldRelativePositionCommand::execute()
{
    runYawPositionController();
    runPitchPositionController();
}

// todo add min/max limiting to the turret here

void TurretWorldRelativePositionCommand::runYawPositionController()
{
    /// \todo fix user input
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
    aruwlib::algorithms::ContiguousFloat min(turretSubsystem->getYawAngle().getValue()
            - 90.0f + Mpu6500::getImuAttitude().yaw, 0.0f, 360.0f);
    aruwlib::algorithms::ContiguousFloat max(turretSubsystem->getYawAngle().getValue()
            + 90.0f + Mpu6500::getImuAttitude().yaw, 0.0f, 360.0f);

    if (min.getValue() < max.getValue())
    {
        currValueImuYawGimbal.setValue(
                aruwlib::algorithms::limitVal<float>(currValueImuYawGimbal.getValue(),
                min.getValue(), max.getValue()));
    }
    else if (min.getValue() != max.getValue())
    {
        currValueImuYawGimbal.setValue(aruwlib::algorithms::limitVal<float>(
                currValueImuYawGimbal.getValue(),
                max.getValue(), min.getValue()));
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

    float positionControllerError = currImuPitchAngle.difference(yawTargetAngle);
    float pidOutput = pitchPid.runController(positionControllerError,
        turretSubsystem->getPitchAngle().getValue());  /// \todo fix imu stuff for pitch

    turretSubsystem->setPitchMotorOutput(pidOutput);
}

}  // namespace control

}  // namespace aruwsrc
