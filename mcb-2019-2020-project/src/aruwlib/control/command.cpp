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
        return commandRequirements.find(requirement) != commandRequirements.end();
    }

    void Command::addSubsystemRequirement(Subsystem* requirement)
    {
        // Ensure the requirement you are trying to add is not already a
        // command requirement.
        if (requirement != nullptr &&
            commandRequirements.find(requirement) == commandRequirements.end()
        ) {
            commandRequirements.insert(requirement);
            // this->comprisedCommandScheduler.registerSubsystem(requirement);  todo test this
        }
    }

    const set<Subsystem*>& Command::getRequirements()
    {
        return commandRequirements;
    }

}  // namespace control

}  // namespace aruwlib
