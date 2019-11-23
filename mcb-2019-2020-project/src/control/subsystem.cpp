#include "src/control/subsystem.hpp"
#include "src/control/command.hpp"
#include "scheduler.hpp"

namespace aruwlib
{

namespace control
{
    Subsystem::Subsystem() : defaultCommand()
    {
        defaultCommand = CommandScheduler::defaultNullCommand;
        CommandScheduler::registerSubsystem(this);
    }

    void Subsystem::setDefaultCommand(modm::SmartPointer command)
    {
        defaultCommand = command;
    }

    modm::SmartPointer Subsystem::getDefaultCommand() const
    {
        return defaultCommand;
    }
}  // namespace control

}  // namespace aruwlib
