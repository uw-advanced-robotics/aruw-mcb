#ifndef __TURRET_WORLD_RELATIVE_POSITION_COMMAND_HPP__
#define __TURRET_WORLD_RELATIVE_POSITION_COMMAND_HPP__

#include <aruwlib/algorithms/contiguous_float.hpp>
#include <aruwlib/control/command.hpp>

#include "aruwsrc/algorithms/turret_pid.hpp"
#include "aruwsrc/control/chassis/chassis_subsystem.hpp"

#include "turret_subsystem.hpp"

namespace aruwsrc
{
namespace turret
{
template <typename Drivers>
class TurretWorldRelativePositionCommand : public aruwlib::control::Command<Drivers>
{
public:
    TurretWorldRelativePositionCommand(
        TurretSubsystem<Drivers> *subsystem,
        chassis::ChassisSubsystem<Drivers> *chassis)
        : turretSubsystem(subsystem),
          chassisSubsystem(chassis),
          yawTargetAngle(TurretSubsystem<Drivers>::TURRET_START_ANGLE, 0.0f, 360.0f),
          currValueImuYawGimbal(0.0f, 0.0f, 360.0f),
          imuInitialYaw(0.0f),
          yawPid(
              YAW_P,
              YAW_I,
              YAW_D,
              YAW_MAX_ERROR_SUM,
              YAW_MAX_OUTPUT,
              YAW_Q_DERIVATIVE_KALMAN,
              YAW_R_DERIVATIVE_KALMAN,
              YAW_Q_PROPORTIONAL_KALMAN,
              YAW_R_PROPORTIONAL_KALMAN),
          pitchPid(
              PITCH_P,
              PITCH_I,
              PITCH_D,
              PITCH_MAX_ERROR_SUM,
              PITCH_MAX_OUTPUT,
              PITCH_Q_DERIVATIVE_KALMAN,
              PITCH_R_DERIVATIVE_KALMAN,
              PITCH_Q_PROPORTIONAL_KALMAN,
              PITCH_R_PROPORTIONAL_KALMAN)
    {
        this->addSubsystemRequirement(subsystem);
    }

    void initialize() override
    {
        imuInitialYaw = Drivers::mpu6500.getYaw();
        yawPid.reset();
        pitchPid.reset();
        yawTargetAngle.setValue(turretSubsystem->getYawTarget());
    }

    bool isFinished() const override { return false; }

    void execute() override
    {
        runYawPositionController();
        runPitchPositionController();
    }

    void end(bool) override
    {
        turretSubsystem->setYawTarget(
            projectWorldRelativeYawToChassisFrame(yawTargetAngle.getValue(), imuInitialYaw));
    }

    const char *getName() const override { return "turret world relative position command"; }

private:
    static constexpr float YAW_P = 4500.0f;
    static constexpr float YAW_I = 0.0f;
    static constexpr float YAW_D = 140.0f;
    static constexpr float YAW_MAX_ERROR_SUM = 0.0f;
    static constexpr float YAW_MAX_OUTPUT = 32000.0f;
    static constexpr float YAW_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_R_DERIVATIVE_KALMAN = 20.0f;
    static constexpr float YAW_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float YAW_R_PROPORTIONAL_KALMAN = 0.0f;

    static constexpr float PITCH_P = 4500.0f;
    static constexpr float PITCH_I = 0.0f;
    static constexpr float PITCH_D = 90.0f;
    static constexpr float PITCH_MAX_ERROR_SUM = 0.0f;
    static constexpr float PITCH_MAX_OUTPUT = 32000.0f;
    static constexpr float PITCH_Q_DERIVATIVE_KALMAN = 1.5f;
    static constexpr float PITCH_R_DERIVATIVE_KALMAN = 20.0f;
    static constexpr float PITCH_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float PITCH_R_PROPORTIONAL_KALMAN = 2.0f;

    static constexpr float USER_YAW_INPUT_SCALAR = 0.75f;
    static constexpr float USER_PITCH_INPUT_SCALAR = 0.5f;

    static constexpr float PITCH_GRAVITY_COMPENSATION_KP = 4000.0f;

    TurretSubsystem<Drivers> *turretSubsystem;
    chassis::ChassisSubsystem<Drivers> *chassisSubsystem;

    aruwlib::algorithms::ContiguousFloat yawTargetAngle;

    aruwlib::algorithms::ContiguousFloat currValueImuYawGimbal;

    float imuInitialYaw;

    aruwsrc::algorithms::TurretPid yawPid;
    aruwsrc::algorithms::TurretPid pitchPid;

    void runYawPositionController()
    {
        turretSubsystem->updateCurrentTurretAngles();

        yawTargetAngle.shiftValue(
            USER_YAW_INPUT_SCALAR * Drivers::controlOperatorInterface.getTurretYawInput());

        // project target angle in world relative to chassis relative to limit the value
        turretSubsystem->setYawTarget(
            projectWorldRelativeYawToChassisFrame(yawTargetAngle.getValue(), imuInitialYaw));

        // project angle that is limited by the subsystem to world relative again to run the
        // controller
        yawTargetAngle.setValue(projectChassisRelativeYawToWorldRelative(
            turretSubsystem->getYawTarget(),
            imuInitialYaw));

        currValueImuYawGimbal.setValue(projectChassisRelativeYawToWorldRelative(
            turretSubsystem->getYawAngle().getValue(),
            imuInitialYaw));

        // position controller based on imu and yaw gimbal angle
        float positionControllerError = currValueImuYawGimbal.difference(yawTargetAngle);
        float pidOutput = yawPid.runController(
            positionControllerError,
            turretSubsystem->getYawVelocity() + Drivers::mpu6500.getGz());

        pidOutput += turretSubsystem->yawFeedForwardCalculation(
            chassisSubsystem->getChassisDesiredRotation());

        turretSubsystem->setYawMotorOutput(pidOutput);
    }
    void runPitchPositionController()
    {
        // limit the yaw min and max angles
        turretSubsystem->setPitchTarget(
            turretSubsystem->getPitchTarget() +
            USER_PITCH_INPUT_SCALAR * Drivers::controlOperatorInterface.getTurretPitchInput());

        // position controller based on turret pitch gimbal and imu data
        float positionControllerError =
            turretSubsystem->getPitchAngle().difference(turretSubsystem->getPitchTarget());

        float pidOutput =
            pitchPid.runController(positionControllerError, turretSubsystem->getPitchVelocity());

        // gravity compensation
        pidOutput +=
            PITCH_GRAVITY_COMPENSATION_KP *
            cosf(aruwlib::algorithms::degreesToRadians(turretSubsystem->getPitchAngleFromCenter()));

        turretSubsystem->setPitchMotorOutput(pidOutput);
    }

    static float projectChassisRelativeYawToWorldRelative(float yawAngle, float imuInitialAngle)
    {
        return yawAngle + Drivers::mpu6500.getYaw() - imuInitialAngle;
    }
    static float projectWorldRelativeYawToChassisFrame(float yawAngle, float imuInitialAngle)
    {
        return yawAngle - Drivers::mpu6500.getYaw() + imuInitialAngle;
    }
};

}  // namespace turret

}  // namespace aruwsrc

#endif
