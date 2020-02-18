#include "command.hpp"
#include "command_scheduler.hpp"

using namespace std;

namespace aruwlib
{

namespace control
{
    // cppcheck-suppress unusedFunction //TODO Remove lint suppression
    bool Command::hasRequirement(Subsystem* requirement) const
    {
        if (requirement == nullptr)
        {
            return false;
        }
        return commandRequirements.find(requirement) != commandRequirements.end();
    }

    void Command::addSubsystemRequirement(Subsystem* requirement)
    {
        if (requirement == nullptr)
        {
            return;
        }
        // Ensure the requirement you are trying to add is not already a
        // command requirement.
        if (requirement != nullptr &&
            commandRequirements.find(requirement) == commandRequirements.end()
        ) {
            commandRequirements.insert(requirement);
            this->comprisedCommandScheduler.registerSubsystem(requirement);
        }
    }

    const set<Subsystem*>& Command::getRequirements()
    {
        return commandRequirements;
    }

}  // namespace control

}  // namespace aruwlib
