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

    void initialize() override {}

    void execute() override;

    void end(bool) override {}

private:
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
