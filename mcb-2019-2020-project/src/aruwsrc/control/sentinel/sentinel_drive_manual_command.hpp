#ifndef __SENTINEL_DRIVE_MANUAL_COMMAND_HPP__
#define __SENTINEL_DRIVE_MANUAL_COMMAND_HPP__

#include <aruwlib/control/command.hpp>

#include "sentinel_drive_subsystem.hpp"

namespace aruwsrc
{
namespace control
{
template <typename Drivers> class SentinelDriveManualCommand : public aruwlib::control::Command
{
public:
    explicit SentinelDriveManualCommand(SentinelDriveSubsystem<Drivers>* subsystem)
        : Command(),
          subsystemSentinelDrive(subsystem)
    {
        addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(subsystem));
    }

    void initialize() override {}

    void execute() override
    {
        subsystemSentinelDrive->setDesiredRpm(
            Drivers::controlOperatorInterface.getSentinelSpeedInput());
    }

    void end(bool) override { subsystemSentinelDrive->setDesiredRpm(0); }

    bool isFinished() const override { return false; }

    const char* getName() const override { return "sentinel drive manual command"; }

private:
    SentinelDriveSubsystem<Drivers>* subsystemSentinelDrive;
};

}  // namespace control

}  // namespace aruwsrc

#endif
