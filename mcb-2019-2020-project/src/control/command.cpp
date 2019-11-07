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

    bool Command::hasRequirement(Subsystem* requirement) const
    {
        bool hasRequirement = false;
        for (int i = commandRequirements->getSize(); i > 0; i--)
        {
            const Subsystem* currSubsystem = commandRequirements->getFront();
            commandRequirements->removeFront();
            commandRequirements->append(currSubsystem);
            if (requirement == commandRequirements->getFront())
            {
                hasRequirement = true;
            }
        }
        return hasRequirement;
    }

    void Command::addSubsystemRequirement(const Subsystem* requirement)
    {
        // TODO(matthew) check for repeated subsystems
        commandRequirements->append(requirement);
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
