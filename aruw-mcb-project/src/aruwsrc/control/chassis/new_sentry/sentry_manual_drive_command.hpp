#ifndef SENTRY_MANUAL_DRIVE_COMMAND_HPP_
#define SENTRY_MANUAL_DRIVE_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/robot/sentry/sentry_control_operator_interface.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"

using namespace aruwsrc::control::sentry;

namespace aruwsrc
{
namespace control::sentry
{

/**
 * A command that controls chassis-relative mecanum drive.
 */
class SentryManualDriveCommand : public tap::control::Command
{
public:
    SentryManualDriveCommand(
        tap::Drivers* drivers,
        SentryControlOperatorInterface* operatorInterface,
        chassis::HolonomicChassisSubsystem* chassis);

    void initialize() override;

    /**
     * Gets remote x, y, and r commands, limits them, applies a rotation ratio between [0, 1]
     * that is inversely proportional to the rotation component to the x and y components of
     * movement, and sets `setDesiredOutput` with the scaled <x, y, r> components.
     */
    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "chassis drive"; }

private:
    tap::Drivers* drivers;
    SentryControlOperatorInterface* operatorInterface;
    chassis::HolonomicChassisSubsystem* chassis;
};  // class SentryManualDriveCommand

}  // namespace aruwsrc::control::sentry

}  // namespace aruwsrc

#endif // SENTRY_MANUAL_DRIVE_COMMAND_HPP_
