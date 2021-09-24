#ifndef TURRET_CHASSIS_RELATIVE_COMMAND_HPP_
#define TURRET_CHASSIS_RELATIVE_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/turret/turret_subsystem_interface.hpp"

namespace tap
{
class Drivers;
}

namespace aruwsrc::control::turret
{
class TurretChassisRelativeCommand : tap::control::Command
{
public:
    TurretChassisRelativeCommand(tap::Drivers *drivers, tap::control::turret::TurretSubsystemInterface *turretSubsystem, const float yawInputScalar, const float pitchInputScalar);

    bool isReady() override { return turretSubsystem->isOnline(); }

    bool isFinished() const override { return !turretSubsystem->isOnline(); }

    const char* getName() const override { return "Turret CR command"; }

    void initialize() override;

    void execute() override;

    void end(bool) override;

private:
    static constexpr float YAW_P = 3800.0f;
    static constexpr float YAW_I = 50.0f;
    static constexpr float YAW_D = 220.0f;
    static constexpr float YAW_MAX_ERROR_SUM = 1000.0f;
    static constexpr float YAW_MAX_OUTPUT = 30000.0f;
    static constexpr float YAW_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_R_DERIVATIVE_KALMAN = 10.0f;
    static constexpr float YAW_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float YAW_R_PROPORTIONAL_KALMAN = 10.0f;

    static constexpr float PITCH_P = 3200.0f;
    static constexpr float PITCH_I = 0.0f;
    static constexpr float PITCH_D = 120.0f;
    static constexpr float PITCH_MAX_ERROR_SUM = 0.0f;
    static constexpr float PITCH_MAX_OUTPUT = 30000.0f;
    static constexpr float PITCH_Q_DERIVATIVE_KALMAN = 1.5f;
    static constexpr float PITCH_R_DERIVATIVE_KALMAN = 47.0f;
    static constexpr float PITCH_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float PITCH_R_PROPORTIONAL_KALMAN = 2.0f;

    static constexpr float USER_YAW_INPUT_SCALAR = 1.5f;
    static constexpr float USER_PITCH_INPUT_SCALAR = 0.6f;

    tap::Drivers *drivers;
    tap::control::turret::TurretSubsystemInterface *turretSubsystem;

    const float yawInputScalar, pitchInputScalar;

    uint32_t prevTime;

    // Pitch/yaw PID controllers
    tap::algorithms::SmoothPid yawPid;
    tap::algorithms::SmoothPid pitchPid;
};
}  // namespace aruwsrc::control::turret

#endif  // TURRET_CHASSIS_RELATIVE_COMMAND_HPP_
