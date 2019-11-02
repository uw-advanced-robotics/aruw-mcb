#include "src/control/command.hpp"
#include "src/control/scheduler.hpp"


namespace aruwlib
{

namespace control
{    
    /**
      * Schedules this command.
      *
      * @param interruptible whether this command can be interrupted by another
      * command that shares one of its requirements
      */
    void Command::schedule(bool interruptible)
    {
        isInterruptiable = interruptible;
        Scheduler::addCommand(this);
    }

    /**
      * Cancels this command.  Will call the command's interrupted() method.
      * Commands will be canceled even if they are not marked as interruptible.
      */
    void Command::cancel(void)
    {
        this->interrupted();
        Scheduler::removeCommand(this);
    }

    /**
      * Whether or not the command is currently scheduled.  Note that this does not
      * detect whether the command is being run by a CommandGroup, only whether it
      * is directly being run by the scheduler.
      *
      * @return Whether the command is scheduled.
      */
    bool Command::isScheduled() const
    {
        return Scheduler::isScheduled(this);
    }

    /**
      * Whether the command requires a given subsystem.  Named "hasRequirement"
      * rather than "requires" to avoid confusion with
      * {@link
      * edu.wpi.first.wpilibj.command.Command#requires(edu.wpi.first.wpilibj.command.Subsystem)}
      *  - this may be able to be changed in a few years.
      *
      * @param requirement the subsystem to inquire about
      * @return whether the subsystem is required
      */
    bool Command::hasRequirement(Subsystem* requirement) const
    {
        modm::LinkedList<Subsystem*> requirements = this->getRequirements();
        for (int i = requirements.getSize(); i >= 0; i--)
        {
            if (requirement == requirements.getFront())
            {
                return true;
            }
        }
        return false;
    }

}  // namespace command

}  // namespace aruwlib
