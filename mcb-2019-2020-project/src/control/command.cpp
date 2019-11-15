#include "src/control/command.hpp"
#include "src/control/scheduler.hpp"

using namespace std;

namespace aruwlib
{

namespace control
{
    Command::Command(bool isInterruptible) : isCommandInterruptiable(isInterruptible)
    {
        cmdRequirements = new set<const Subsystem*>();
        commandRequirements = modm::SmartPointer(new set<const Subsystem*>);
    }

    bool Command::hasRequirement(const Subsystem* requirement) const
    {
        return getRequirements().find(requirement) != getRequirements().end();
    }

    void Command::addSubsystemRequirement(const Subsystem* requirement)
    {
        // Ensure the requirement you are trying to add is not already a
        // command requirement.
        if (getRequirements().find(requirement) == getRequirements().end())
        {
            getRequirementsModifiable()->insert(requirement);
        }
    }

    const set<const Subsystem*>& Command::getRequirements() const
    {
        return *cmdRequirements;
    }

    set<const Subsystem*>* Command::getRequirementsModifiable()
    {
        return cmdRequirements;
        return reinterpret_cast<set<const Subsystem*>*>(commandRequirements.getPointer());

    }

    bool Command::isInterruptible() const
    {
        return isCommandInterruptiable;
    }
}  // namespace control

}  // namespace aruwlib
