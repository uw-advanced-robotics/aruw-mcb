#include "src/control/subsystem.hpp"
#include "src/control/command.hpp"
#include "scheduler.hpp"

namespace aruwlib
{

namespace control
{
    Subsystem::Subsystem() : defaultCommand()
    {
        CommandScheduler::registerSubsystem(this);
    }

    void Subsystem::SetDefaultCommand(modm::SmartPointer& command)
    {
        defaultCommand = command;
    }

    modm::SmartPointer Subsystem::GetDefaultCommand() const
    {
        return defaultCommand;
    }
}  // namespace control

}  // namespace aruwlib
