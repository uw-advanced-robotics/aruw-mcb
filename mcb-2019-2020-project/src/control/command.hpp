/**
 * A generic extendable class for implementing a command. Each 
 * command is attached to a subsystem. To create a new command,
 * extend the Command class and instantiate the virtual functions
 * in this class.
 */

#ifndef __COMMAND_HPP__
#define __COMMAND_HPP__

#include "rm-dev-board-a/board.hpp"
#include "src/control/subsystem.hpp"
#include <modm/container/dynamic_array.hpp>
#include <modm/container/smart_pointer.hpp>

namespace aruwlib
{

namespace control
{

class Command {
 public:
    Command()
    {
       commandRequirements = modm::SmartPointer(new modm::DynamicArray
          <const Subsystem*>(SUBSYSTEM_REQUIREMENT_LIST_SIZE));
    }

    void initCommand()
    {
    }

    ~Command()
    {
    }

    /**
     * Specifies the set of subsystems used by this command.  Two commands cannot
     * use the same subsystem at the same time.  If the command is scheduled as
     * interruptible and another command is scheduled that shares a requirement,
     * the command will be interrupted.  Else, the command will not be scheduled.
     * If no subsystems are required, return an empty set.
     *
     * The generic Command class contains a list of the requrements. The user
     * should add requirements to this list accordingly (typically in the constructor
     * of a class extending the Command class).
     *
     * @return the set of subsystems that are required
     */
    const modm::DynamicArray<const Subsystem*>& getRequirements() const;

    /**
     * Schedules this command.
     *
     * @param interruptible whether this command can be interrupted by another
     * command that shares one of its requirements
     */
    void schedule(bool interruptible);

    /**
     * Schedules this command, defaulting to interruptible.
     */
    void schedule()
    {
       schedule(true);
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
    bool hasRequirement(const Subsystem* requirement) const;

    /**
     * Adds the required subsystem to a list of required subsystems
     */
    void addSubsystemRequirement(const Subsystem* requirement);

    /**
     * Returns isCommandInterruptiable
     */
    bool isInterruptiable(void) const;

    /**
     * The initial subroutine of a command.  Called once when the command is
     * initially scheduled.
     */
    virtual void initialize(void) = 0;

    /**
     * The main body of a command.  Called repeatedly while the command is
     * scheduled.
     */
    virtual void execute(void) = 0;

    /**
     * The action to take when the command ends.  Called when either the command
     * finishes normally, or when it interrupted/canceled.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    virtual void end(bool interrupted) = 0;

    /**
     * Whether the command has finished.  Once a command finishes, the scheduler
     * will call its end() method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    virtual bool isFinished(void)
    {
       return false;
    }

 private:
    const int SUBSYSTEM_REQUIREMENT_LIST_SIZE = 5;

    bool isCommandInterruptiable = true;

    // contains pointers to const Subsystem pointers that this command requires
    modm::SmartPointer commandRequirements;

    /**
     * An internal helper method that returns the contents of the
     * commandRequirements smart pointer in a modifiable form
     */
    modm::DynamicArray<const Subsystem*>* getRequirementsModifiable(void);
};

}  // namespace control

}  // namespace aruwlib

#endif
