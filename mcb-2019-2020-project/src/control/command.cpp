#include "src/control/command.hpp"
#include "src/control/scheduler.hpp"

using namespace std;

namespace aruwlib
{

namespace control
{
    Command::Command(bool isInterruptiable) : isCommandInterruptiable(isInterruptiable)
    {
        commandRequirements = modm::SmartPointer(new set<const Subsystem*>);
        // commandRequirements = modm::SmartPointer(new modm::DynamicArray
        //     <const Subsystem*>(SUBSYSTEM_REQUIREMENT_LIST_SIZE));
    }

    void Command::schedule()
    {
        // schedule the command add command to scheduler TODO(matthew)
    }

    bool Command::hasRequirement(const Subsystem* requirement) const
    {
        getRequirements().find(requirement);
        // for (const Subsystem* subsystemRequirement : getRequirements())
        // {
        //     if (requirement == subsystemRequirement)
        //     {
        //         return true;
        //     }
        // }
        // return false;
    }

    void Command::addSubsystemRequirement(const Subsystem* requirement)
    {
        // Ensure the requirement you are trying to add is not already a
        // command requirement.
        for (const Subsystem* subsystemRequirement : getRequirements())
        {
            if (subsystemRequirement == requirement)
            {
                return;
            }
        }
        getRequirementsModifiable()->append(requirement);
    }

    const set<const Subsystem*>& Command::getRequirements() const
    {
        return *reinterpret_cast<const set<const Subsystem*>*>(
            commandRequirements.getPointer());
    }

    // const modm::DynamicArray<const Subsystem*>& Command::getRequirements() const
    // {
    //     return *reinterpret_cast<const modm::DynamicArray<const Subsystem*>*>(
    //         commandRequirements.getPointer());
    // }

    set<const Subsystem*>* Command::getRequirementsModifiable()
    {
        return reinterpret_cast<set<const Subsystem*>*>(commandRequirements.getPointer());

    }
    

    // modm::DynamicArray<const Subsystem*>* Command::getRequirementsModifiable()
    // {
    //     return reinterpret_cast<modm::DynamicArray<const Subsystem*>*>(
    //         commandRequirements.getPointer());
    // }

    bool Command::isInterruptiable() const
    {
        return isCommandInterruptiable;
    }
}  // namespace control

}  // namespace aruwlib
