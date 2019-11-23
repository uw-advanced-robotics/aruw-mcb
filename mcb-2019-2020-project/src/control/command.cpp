#include "src/control/command.hpp"
#include "src/control/scheduler.hpp"

using namespace std;

namespace aruwlib
{

namespace control
{
    bool Command::hasRequirement(const Subsystem* requirement) const
    {
        return commandRequirements.find(requirement) != commandRequirements.end();
    }

    void Command::addSubsystemRequirement(const Subsystem* requirement)
    {
        // Ensure the requirement you are trying to add is not already a
        // command requirement.
        if (requirement != nullptr &&
            getRequirements().find(requirement) == getRequirements().end())
        {
            getRequirementsModifiable()->insert(requirement);
        }
    }

    const set<const Subsystem*>& Command::getRequirements() const
    {
        return commandRequirements;
    }

    set<const Subsystem*>* Command::getRequirementsModifiable()
    {
        return &commandRequirements;
    }
}  // namespace control

}  // namespace aruwlib
