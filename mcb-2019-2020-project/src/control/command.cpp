#include "src/control/command.hpp"
#include "src/control/scheduler.hpp"

namespace aruwlib
{

namespace control
{    
    void Command::schedule(bool interruptible)
    {
        m_isInterruptiable = interruptible;
        Scheduler::addCommand(this);
    }

    void Command::cancel(void)
    {
        this->interrupted();
        Scheduler::removeCommand(this);
    }

    bool Command::isScheduled() const
    {
        return Scheduler::isScheduled(this);
    }

    bool Command::hasRequirement(const Subsystem* requirement) const
    {
        bool hasRequirement = false;
        for (int i = commandRequirements->getSize(); i > 0; i--)
        {
            const Subsystem* currSubsystem = commandRequirements->getFront();
            if (requirement == commandRequirements->getFront())
            {
                hasRequirement = true;
            }
            commandRequirements->removeFront();
            commandRequirements->append(currSubsystem);
        }
        return hasRequirement;
    }

    bool Command::addSubsystemRequirement(const Subsystem* requirement)
    {
        for (int i = commandRequirements->getSize(); i > 0; i--)
        {
            const Subsystem* currSubsystem = commandRequirements->getFront();
            if (currSubsystem == requirement)
            {
                commandRequirements->removeFront();
                commandRequirements->append(currSubsystem);
                return false;
            }
            commandRequirements->removeFront();
            commandRequirements->append(currSubsystem);
        }
        commandRequirements->append(requirement);
        return true;
    }

    modm::LinkedList<const Subsystem*>& Command::getRequirements() const
    {
        return *commandRequirements;
    }

    bool Command::isInterruptiable() const
    {
        return m_isInterruptiable;
    }
}  // namespace command

}  // namespace aruwlib
