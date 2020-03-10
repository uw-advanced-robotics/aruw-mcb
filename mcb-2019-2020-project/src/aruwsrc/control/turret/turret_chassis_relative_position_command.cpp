#include "turret_chassis_relative_position_command.hpp"
#include "src/aruwlib/communication/sensors/mpu6500/mpu6500.hpp"
#include "src/aruwlib/communication/remote.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/control/control_operator_interface.hpp"

using namespace aruwlib::sensors;
using namespace aruwlib;

namespace aruwsrc
{

namespace control
{

TurretChassisRelativePositionCommand::TurretChassisRelativePositionCommand(
    TurretSubsystem *subsystem
) :
    turretSubsystem(subsystem),
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

void TurretChassisRelativePositionCommand::initialize()
{
    yawPid.reset();
    pitchPid.reset();
}

void TurretChassisRelativePositionCommand::execute()
{
    turretSubsystem->updateCurrentTurretAngles();
    runYawPositionController();
    runPitchPositionController();
}

void TurretChassisRelativePositionCommand::runYawPositionController()
{
    turretSubsystem->setYawTarget(turretSubsystem->getYawTarget()
            + USER_YAW_INPUT_SCALAR
            * aruwlib::control::ControlOperatorInterface::getTurretYawInput());
    float positionControllerError = turretSubsystem->getYawAngle()
            .difference(turretSubsystem->getYawTarget());
    float pidOutput = yawPid.runController(positionControllerError,
            turretSubsystem->getYawVelocity());

    turretSubsystem->setYawMotorOutput(pidOutput);
}

void TurretChassisRelativePositionCommand::runPitchPositionController()
{
    // limit the yaw min and max angles
    turretSubsystem->setPitchTarget(turretSubsystem->getPitchTarget()
            + USER_PITCH_INPUT_SCALAR
            * aruwlib::control::ControlOperatorInterface::getTurretPitchInput());

    // position controller based on turret pitch gimbal and imu data
    float positionControllerError = turretSubsystem->getPitchAngle().difference(
            turretSubsystem->getPitchTarget());

    float pidOutput = pitchPid.runController(positionControllerError,
        turretSubsystem->getPitchVelocity());

    // gravity compensation
    #if defined(TARGET_SOLDIER)
    pidOutput += PITCH_GRAVITY_COMPENSATION_KP * cosf(aruwlib::algorithms::degreesToRadians(
            turretSubsystem->getPitchAngleFromCenter()));
    #elif defined(TARGET_SENTRY)
    // best fit curve when plotting motor output to hold the pitch motor up vs pitch angle
    pidOutput += 1.7736f * powf(turretSubsystem->getPitchAngle().getValue(), 2.0f)
            + 11.802 * turretSubsystem->getPitchAngle().getValue()
            - 23417.0f;
    #endif

    turretSubsystem->setPitchMotorOutput(pidOutput);
}

// NOLINTNEXTLINE
void TurretChassisRelativePositionCommand::end(bool)
{}

}  // namespace control

}  // namespace aruwsrc
