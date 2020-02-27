#include <modm/math/geometry/vector2.hpp>
#include "turret_world_relative_position_command.hpp"
#include "src/aruwlib/communication/sensors/mpu6500/mpu6500.hpp"
#include "src/aruwlib/communication/remote.hpp"  /// \todo fix and remove remote from command
#include "src/aruwlib/algorithms/math_user_utils.hpp"

using namespace aruwlib::sensors;
using namespace aruwlib;

namespace aruwsrc
{

namespace control
{

TurretWorldRelativePositionCommand::TurretWorldRelativePositionCommand(
    TurretSubsystem *subsystem, chassis::ChassisSubsystem *chassis
) :
    turretSubsystem(subsystem),
    chassisSubsystem(chassis),
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
    /// \todo uncomment when the agitator code has been merged in, has reset method there
    // yawPid.reset();
    // pitchPid.reset();
    /// \todo decide if this is the best thing to do
    yawTargetAngle.setValue(turretSubsystem->getYawAngle().getValue());
    pitchTargetAngle.setValue(turretSubsystem->getPitchAngle().getValue());
}

void TurretWorldRelativePositionCommand::execute()
{
    runYawPositionController();
    runPitchPositionController();
}

float gains = 2.75f;
float gains2 = 30.0f;
float gains3 = 1.0f;
float derivative = 0.0f;
float prevRotationDesired = 0.0f;
float wchassis = 0.0f;

void TurretWorldRelativePositionCommand::runYawPositionController()
{
    // calculate the desired user angle in world reference frame
    // if user does not want to move the turret, recalibrate the imu initial value
    yawTargetAngle.shiftValue(getUserTurretYawInput());
    // the position controller is in world reference frame
    // (i.e. add imu yaw to current encoder value)
    currValueImuYawGimbal.setValue(turretSubsystem->getYawAngle().getValue()
            + Mpu6500::getImuAttitude().yaw - imuInitialYaw);

    // limit the yaw min and max angles
    aruwlib::algorithms::ContiguousFloat min(
        turretSubsystem->TURRET_YAW_MIN_ANGLE + Mpu6500::getImuAttitude().yaw, 0.0f, 360.0f);
    aruwlib::algorithms::ContiguousFloat max(
        turretSubsystem->TURRET_YAW_MAX_ANGLE + Mpu6500::getImuAttitude().yaw, 0.0f, 360.0f);
    yawTargetAngle.limitValue(min, max);

    float positionControllerError = currValueImuYawGimbal.difference(yawTargetAngle);
    float pidOutput = yawPid.runController(positionControllerError,
        turretSubsystem->getYawVelocity() + Mpu6500::getGz());  /// \todo fix gz in mpu6500 class


    wchassis = chassisSubsystem->getChassisDesiredRotation();
    pidOutput += gains * chassisSubsystem->getChassisDesiredRotation()
            * (fabsf(sin(turretSubsystem->getYawAngleFromCenter()
            * aruwlib::algorithms::PI / 180.0f)) + gains3);
    derivative = gains2 * 0.154f * (chassisSubsystem->getChassisDesiredRotation()
            - prevRotationDesired) + (1.0f - 0.154f) * derivative;
    pidOutput += derivative;


    prevRotationDesired = chassisSubsystem->getChassisDesiredRotation();
    turretSubsystem->setYawMotorOutput(pidOutput);
}

void TurretWorldRelativePositionCommand::runPitchPositionController()
{
    // calculate the desired user angle in world reference frame
    // if user does not want to move the turret, recalibrate the imu initial value
    pitchTargetAngle.shiftValue(getUserTurretPitchInput());
    // the position controller is in world reference frame
    // (i.e. add imu yaw to current encoder value)
    currImuPitchAngle.setValue(turretSubsystem->getPitchAngle().getValue() + calcPitchImuOffset());

    // limit the yaw min and max angles
    aruwlib::algorithms::ContiguousFloat min(
        turretSubsystem->TURRET_PITCH_MIN_ANGLE + calcPitchImuOffset(), 0.0f, 360.0f);
    aruwlib::algorithms::ContiguousFloat max(
        turretSubsystem->TURRET_PITCH_MAX_ANGLE + calcPitchImuOffset(), 0.0f, 360.0f);
    pitchTargetAngle.limitValue(min, max);

    float positionControllerError = currImuPitchAngle.difference(pitchTargetAngle);
    float pidOutput = pitchPid.runController(positionControllerError,
        turretSubsystem->getPitchVelocity());  /// \todo fix imu stuff for pitch

    turretSubsystem->setPitchMotorOutput(pidOutput);
}

float TurretWorldRelativePositionCommand::calcPitchImuOffset()
{
    if (Mpu6500::getImuAttitude().pitch < 45.0f && Mpu6500::getImuAttitude().roll < 45.0f)
    {
        // for following vector calculations
        // i direction perpendicular to foward direction of chassis
        // j direction facing foward on chassis
        modm::Vector2f imuVector = {Mpu6500::getImuAttitude().pitch,
                                    -Mpu6500::getImuAttitude().roll};
        modm::Vector2f turretVector = {
            cos(aruwlib::algorithms::degreesToRadians(turretSubsystem->getYawAngle().getValue())),
            sin(aruwlib::algorithms::degreesToRadians(turretSubsystem->getYawAngle().getValue()))
        };

        // unit vector
        imuVector.normalize();

        float yawImuDot = imuVector.dot(turretVector);

        return yawImuDot * Mpu6500::getTiltAngle();
    }
    else
    {
        return 0.0f;
    }
}

float TurretWorldRelativePositionCommand::getUserTurretYawInput()
{
    float userVelocity = -static_cast<float>(Remote::getChannel(Remote::Channel::RIGHT_HORIZONTAL))
            * USER_REMOTE_YAW_SCALAR
            + static_cast<float>(Remote::getMouseX()) * USER_MOUSE_YAW_SCALAR;
    /// \todo fix low pass
    lowPassUserVelocityYaw = USER_INPUT_LOW_PASS_ALPHA * userVelocity
            + (1 - USER_INPUT_LOW_PASS_ALPHA) * lowPassUserVelocityYaw;
    return lowPassUserVelocityYaw;
}

float TurretWorldRelativePositionCommand::getUserTurretPitchInput()
{
    float userVelocity = static_cast<float>(Remote::getChannel(Remote::Channel::RIGHT_VERTICAL))
            * USER_REMOTE_PITCH_SCALAR
            - static_cast<float>(aruwlib::Remote::getMouseX()) * USER_MOUSE_PITCH_SCALAR;
    /// \todo fix low pass
    lowPassUserVelocityPitch = USER_INPUT_LOW_PASS_ALPHA * userVelocity
            + (1 - USER_INPUT_LOW_PASS_ALPHA) * lowPassUserVelocityPitch;
    return lowPassUserVelocityPitch;
}

}  // namespace control

}  // namespace aruwsrc
