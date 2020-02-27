#include <modm/math/geometry/vector2.hpp>
#include "turret_world_relative_position_command.hpp"
#include "src/aruwlib/communication/sensors/mpu6500/mpu6500.hpp"
#include "src/aruwlib/communication/remote.hpp"
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
    // or should I do this. I think for startup, this should be used, but for toggling
    // between CV and user control, the other one should be used
    yawTargetAngle.setValue(TurretSubsystem::TURRET_START_ANGLE);
    pitchTargetAngle.setValue(TurretSubsystem::TURRET_START_ANGLE);
}

void TurretWorldRelativePositionCommand::execute()
{
    runYawPositionController();
    runPitchPositionController();
}

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

    // position controller based on imu and yaw gimbal angle
    float positionControllerError = currValueImuYawGimbal.difference(yawTargetAngle);
    float pidOutput = yawPid.runController(positionControllerError,
        turretSubsystem->getYawVelocity() + Mpu6500::getGz());  /// \todo fix gz in mpu6500 class

    // feed forward controller based on desired chassis wheel rotation
    float chassisRotationProportional = FEED_FORWARD_KP
            * chassisSubsystem->getChassisDesiredRotation()
            * (fabsf(FEED_FORWARD_SIN_GAIN * sin(turretSubsystem->getYawAngleFromCenter()
            * aruwlib::algorithms::PI / 180.0f)) + 1.0f);

    chassisRotationDerivative = aruwlib::algorithms::lowPassFilter(chassisRotationDerivative,
            chassisSubsystem->getChassisDesiredRotation() - prevChassisRotationDesired,
            FEED_FORWARD_DERIVATIVE_LOW_PASS);

    float chassisRotationFeedForward = aruwlib::algorithms::limitVal<float>(
            chassisRotationProportional + FEED_FORWARD_KD * chassisRotationDerivative,
            -FEED_FORWARD_MAX_OUTPUT, FEED_FORWARD_MAX_OUTPUT);

    // don't do feed forward if it is trying to go past turret bounds
    if ((chassisRotationFeedForward > 0.0f
        && turretSubsystem->getYawAngle().getValue() > TurretSubsystem::TURRET_YAW_MAX_ANGLE)
        || (chassisRotationFeedForward < 0.0f
        && turretSubsystem->getYawAngle().getValue() < TurretSubsystem::TURRET_YAW_MIN_ANGLE))
    {
        chassisRotationFeedForward = 0.0f;
    }

    pidOutput += chassisRotationFeedForward;
    prevChassisRotationDesired = chassisSubsystem->getChassisDesiredRotation();

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

    // position controller based on turret pitch gimbal and imu data
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
    lowPassUserVelocityYaw = aruwlib::algorithms::lowPassFilter(lowPassUserVelocityYaw,
            userVelocity, USER_INPUT_LOW_PASS_ALPHA);
    return lowPassUserVelocityYaw;
}

float TurretWorldRelativePositionCommand::getUserTurretPitchInput()
{
    float userVelocity = static_cast<float>(Remote::getChannel(Remote::Channel::RIGHT_VERTICAL))
            * USER_REMOTE_PITCH_SCALAR
            - static_cast<float>(aruwlib::Remote::getMouseX()) * USER_MOUSE_PITCH_SCALAR;
    lowPassUserVelocityPitch = aruwlib::algorithms::lowPassFilter(lowPassUserVelocityPitch,
            userVelocity, USER_INPUT_LOW_PASS_ALPHA);
    return lowPassUserVelocityPitch;
}

}  // namespace control

}  // namespace aruwsrc
