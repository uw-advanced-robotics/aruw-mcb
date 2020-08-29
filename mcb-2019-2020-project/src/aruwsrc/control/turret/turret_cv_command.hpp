#ifndef TURRET_CV_COMMAND_HPP_
#define TURRET_CV_COMMAND_HPP_

#include <aruwlib/algorithms/contiguous_float.hpp>
#include <aruwlib/architecture/timeout.hpp>
#include <aruwlib/communication/serial/xavier_serial.hpp>
#include <aruwlib/control/command.hpp>

#include "aruwsrc/algorithms/turret_pid.hpp"
#include "aruwsrc/control/chassis/chassis_subsystem.hpp"

#include "turret_subsystem.hpp"

namespace aruwsrc
{
namespace turret
{
template <typename Drivers> class TurretCVCommand : public aruwlib::control::Command
{
public:
    explicit TurretCVCommand(TurretSubsystem<Drivers> *subsystem);

    void initialize() override
    {
        sendRequestTimer.restart(TIME_BETWEEN_CV_REQUESTS);
        Drivers::xavierSerial.beginTargetTracking();
        yawPid.reset();
        pitchPid.reset();
    }

    bool isFinished() const override { return false; }

    void execute() override
    {
        aruwlib::serial::TurretAimData cvData;
        if (Drivers::xavierSerial.getLastAimData(&cvData))
        {
            if (cvData.hasTarget)
            {
                turretSubsystem->setYawTarget(cvData.yaw);
                turretSubsystem->setPitchTarget(cvData.pitch);
            }
        }
        else if (sendRequestTimer.isExpired())
        {
            Drivers::xavierSerial.beginTargetTracking();
        }
        runYawPositionController();
        runPitchPositionController();
    }

    void end(bool isInterrupted) override { Drivers::xavierSerial.stopTargetTracking(); }

    const char *getName() const override { return "turret cv command"; }

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

    static constexpr uint32_t TIME_BETWEEN_CV_REQUESTS = 1000;

    TurretSubsystem<Drivers> *turretSubsystem;

    aruwlib::algorithms::ContiguousFloat yawTargetAngle;
    aruwlib::algorithms::ContiguousFloat pitchTargetAngle;

    aruwsrc::algorithms::TurretPid yawPid;
    aruwsrc::algorithms::TurretPid pitchPid;

    aruwlib::arch::MilliTimeout sendRequestTimer;

    void runYawPositionController()
    {
        // position controller based on gimbal angle
        float positionControllerError =
            turretSubsystem->getYawAngle().difference(turretSubsystem->getYawTarget());
        float pidOutput =
            yawPid.runController(positionControllerError, turretSubsystem->getYawVelocity());

        turretSubsystem->setYawMotorOutput(pidOutput);
    }

    void runPitchPositionController()
    {
        // position controller based on turret pitch gimbal
        float positionControllerError =
            turretSubsystem->getPitchAngle().difference(turretSubsystem->getPitchTarget());
        float pidOutput =
            pitchPid.runController(positionControllerError, turretSubsystem->getPitchVelocity());

        turretSubsystem->setPitchMotorOutput(pidOutput);
    }
};  // class TurretCvCommand

}  // namespace turret

}  // namespace aruwsrc

#endif  // TURRET_CV_COMMAND_HPP_
