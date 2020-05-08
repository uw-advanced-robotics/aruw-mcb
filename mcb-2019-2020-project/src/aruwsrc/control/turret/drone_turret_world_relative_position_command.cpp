#include "drone_turret_world_relative_position_command.hpp"
#include <aruwlib/communication/sensors/mpu6500/mpu6500.hpp>
#include <aruwlib/communication/remote.hpp>
#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/control/control_operator_interface.hpp>

using namespace aruwlib::sensors;
using namespace aruwlib;

namespace aruwsrc
{

namespace turret
{

DroneTurretWorldRelativePositionCommand::DroneTurretWorldRelativePositionCommand(
    DroneTurretSubsystem *subsystem) :
    turretSubsystem(subsystem),
    yawTargetAbsoluteAngle(DroneTurretSubsystem::TURRET_YAW_START_ANGLE, 0.0f, 360.0f),
    pitchTargetAbsoluteAngle(DroneTurretSubsystem::TURRET_PITCH_START_ANGLE, 0.0f, 360.0f),
    absoluteYawAngle(0.0f, 0.0f, 360.0f),
    absolutePitchAngle(0.0f, 0.0f, 360.0f),
    imuInitialYaw(0.0f),
    imuInitialPitch(0.0f),
    yawPid(
        YAW_P,
        YAW_I,
        YAW_D,
        YAW_MAX_ERROR_SUM,
        YAW_MAX_OUTPUT,
        YAW_Q_DERIVATIVE_KALMAN,
        YAW_R_DERIVATIVE_KALMAN,
        YAW_Q_PROPORTIONAL_KALMAN,
        YAW_R_PROPORTIONAL_KALMAN
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

void DroneTurretWorldRelativePositionCommand::initialize()
{
    imuInitialYaw = Mpu6500::getImuAttitude().yaw;
    imuInitialPitch = Mpu6500::getImuAttitude().pitch;
    yawPid.reset();
    pitchPid.reset();
    yawTargetAbsoluteAngle.setValue(turretSubsystem->getYawTarget());
    pitchTargetAbsoluteAngle.setValue(turretSubsystem->getPitchTarget());
}

void DroneTurretWorldRelativePositionCommand::execute()
{
    turretSubsystem->updateCurrentTurretAngles();
    runYawPositionController();
    runPitchPositionController();
}

void DroneTurretWorldRelativePositionCommand::runYawPositionController()
{
    absoluteYawAngle.setValue(
            projectChassisRelativeYawToWorldRelative(
                    turretSubsystem->getYawAngle().getValue(), imuInitialYaw));

    yawTargetAbsoluteAngle.shiftValue(USER_YAW_INPUT_SCALAR
            * aruwlib::control::ControlOperatorInterface::getTurretYawInput());

    // project target angle in world relative to chassis relative to limit the value
    turretSubsystem->setYawTarget(projectWorldRelativeYawToChassisFrame(
            yawTargetAbsoluteAngle.getValue(), imuInitialYaw));

    // project angle that is limited by the subsystem to world relative again to run the controller
    yawTargetAbsoluteAngle.setValue(projectChassisRelativeYawToWorldRelative(
            turretSubsystem->getYawTarget(), imuInitialYaw));

    // position controller based on imu and yaw gimbal angle
    float positionControllerError = absoluteYawAngle.difference(yawTargetAbsoluteAngle);
    float pidOutput = yawPid.runController(positionControllerError,
            turretSubsystem->getYawVelocity() + Mpu6500::getGz());

    turretSubsystem->setYawMotorOutput(pidOutput);
}

void DroneTurretWorldRelativePositionCommand::runPitchPositionController()
{
    absolutePitchAngle.setValue(
            projectChassisRelativePitchToWorldRelative(
                    turretSubsystem->getPitchAngle().getValue(), imuInitialPitch));

    pitchTargetAbsoluteAngle.shiftValue(USER_PITCH_INPUT_SCALAR
            * aruwlib::control::ControlOperatorInterface::getTurretPitchInput());
    
    turretSubsystem->setPitchTarget(projectWorldRelativePitchToChassisFrame(
            pitchTargetAbsoluteAngle.getValue(), imuInitialPitch));
    
    pitchTargetAbsoluteAngle.setValue(projectChassisRelativePitchToWorldRelative(
            turretSubsystem->getPitchTarget(), imuInitialPitch));
    
    // position controller based on turret pitch gimbal and imu data
    float positionControllerError = absolutePitchAngle.difference(pitchTargetAbsoluteAngle);

    float pidOutput = pitchPid.runController(positionControllerError,
        turretSubsystem->getPitchVelocity() - Mpu6500::getGx());

    // gravity compensation
    pidOutput -= PITCH_GRAVITY_COMPENSATION_KP * cosf(aruwlib::algorithms::degreesToRadians(
            turretSubsystem->getPitchAngleFromCenter()));

    turretSubsystem->setPitchMotorOutput(pidOutput);
}

// NOLINTNEXTLINE
void DroneTurretWorldRelativePositionCommand::end(bool)
{
    turretSubsystem->setYawTarget(projectWorldRelativeYawToChassisFrame(yawTargetAbsoluteAngle.getValue(),
            imuInitialYaw));
    turretSubsystem->setPitchTarget(projectWorldRelativePitchToChassisFrame(pitchTargetAbsoluteAngle.getValue(),
            imuInitialPitch));
}

float DroneTurretWorldRelativePositionCommand::projectChassisRelativeYawToWorldRelative(
    float yawAngle,
    float imuInitialAngle
) {
    return yawAngle + aruwlib::sensors::Mpu6500::getImuAttitude().yaw - imuInitialAngle;
}

float DroneTurretWorldRelativePositionCommand::projectWorldRelativeYawToChassisFrame(
    float yawAngle,
    float imuInitialAngle
) {
    return yawAngle - aruwlib::sensors::Mpu6500::getImuAttitude().yaw + imuInitialAngle;
}

float DroneTurretWorldRelativePositionCommand::projectChassisRelativePitchToWorldRelative(
    float pitchAngle,
    float imuInitialAngle
) {
    return pitchAngle - aruwlib::sensors::Mpu6500::getImuAttitude().roll - imuInitialAngle;
}

float DroneTurretWorldRelativePositionCommand::projectWorldRelativePitchToChassisFrame(
    float pitchAngle,
    float imuInitialAngle
) {
    return pitchAngle + aruwlib::sensors::Mpu6500::getImuAttitude().roll + imuInitialAngle;
}

}  // namespace turret

}  // namespace aruwsrc
