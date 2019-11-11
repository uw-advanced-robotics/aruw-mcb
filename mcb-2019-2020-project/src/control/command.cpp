#include "src/control/command.hpp"
#include "src/control/scheduler.hpp"

namespace aruwlib
{

namespace control
{
    void Command::schedule(bool interruptible)
    {
        m_isInterruptiable = interruptible;
        isCommandScheduled = Scheduler::addCommand(this);
    }

    void Command::cancel(void)
    {
        this->interrupted();
        Scheduler::removeCommand(this);
    }

    bool Command::isScheduled() const
    {
        return isCommandScheduled;
        // return Scheduler::isScheduled(this);
    }

    bool Command::hasRequirement(const Subsystem* requirement) const
    {
        for (int i = 0; i < static_cast<int>(getRequirements().getSize()); i++)
        {
            if (requirement == getRequirements()[i])
            {
                return true;
            }
        }
        return false;
    }

    bool Command::addSubsystemRequirement(const Subsystem* requirement)
    {

        // Insure the requirement you are trying to add is not already a
        // command requirement.
        for (int i = 0; i < static_cast<int>(getRequirements().getSize()); i++)
        {
            if (getRequirements()[i] == requirement)
            {
                return false;
            }
        }
        (*reinterpret_cast<modm::DynamicArray<const Subsystem*>*>(
            commandRequirements2.getPointer())).append(requirement);
        return true;
    }

    const modm::DynamicArray<const Subsystem*>& Command::getRequirements() const
    {
        return *reinterpret_cast<const modm::DynamicArray<const Subsystem*>*>(commandRequirements2.getPointer());
    }

    bool Command::isInterruptiable() const
    {
        return m_isInterruptiable;
    }
}  // namespace control

}  // namespace aruwlib
