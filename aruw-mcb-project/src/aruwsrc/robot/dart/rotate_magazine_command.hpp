#ifndef ROTATE_MAGAZINE_COMMAND_HPP_
#define ROTATE_MAGAZINE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "dart_reloader_subsystem.hpp"


namespace aruwsrc::robot::dart
{
class RotateMagazineCommand : public tap::control::Command
{
public:
    RotateMagazineCommand(DartReloaderSubsystem &subsystem);
    void initialize() override;
    void execute() override;
    void end(bool) override;
    bool isFinished() const override;
    const char *getName() const override { return "ROTATE MAGAZINE"; }

private:
    DartReloaderSubsystem &subsystem;
};
}  // namespace aruwsrc::robot::dart
#endif  // ROTATE_MAGAZINE_COMMAND_HPP_