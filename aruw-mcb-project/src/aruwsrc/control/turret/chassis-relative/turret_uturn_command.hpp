#ifndef TURRET_UTURN_COMMAND_HPP_
#define TURRET_UTURN_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/control/turret/turret_subsystem_interface.hpp"

namespace tap
{
class Drivers;
}

namespace aruwsrc::control::turret
{
class TurretUTurnCommand : tap::control::Command
{
public:
    TurretUTurnCommand(
        tap::control::turret::TurretSubsystemInterface *turretSubsystem,
        const float targetOffsetToTurn);

    bool isReady() override { return turretSubsystem->isOnline(); }

    bool isFinished() const override { return true; }

    const char *getName() const override { return "Turret uturn command"; }

    void initialize() override
    {
        turretSubsystem->setYawSetpoint(
            turretSubsystem->getCurrentYawValue().getValue() + targetOffsetToTurn);
    }

    void execute() override {}

    void end(bool) override {}

private:
    tap::control::turret::TurretSubsystemInterface *turretSubsystem;
    const float targetOffsetToTurn;
};
}  // namespace aruwsrc::control::turret

#endif  // TURRET_UTURN_COMMAND_HPP_
