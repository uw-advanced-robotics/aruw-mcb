/**
 * A generic extendable class for implementing a command. Each 
 * command is attached to a subsystem. To create a new command,
 * extend the Command class and instantiate the firtual functions
 * in this class.
 */

#ifndef __COMMAND_HPP__
#define __COMMAND_HPP__

#include "rm-dev-board-a/board.hpp"
#include <modm/container/linked_list.hpp>
#include "src/control/subsystem.hpp"

namespace aruwlib
{

namespace control
{

class Command {
 public:    

    Command()
    {
       commandRequirements = new modm::LinkedList<Subsystem*>();
    }

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
    virtual bool isFinished(void) { return false; }

    /**
     * Specifies the set of subsystems used by this command.  Two commands cannot
     * use the same subsystem at the same time.  If the command is scheduled as
     * interruptible and another command is scheduled that shares a requirement,
     * the command will be interrupted.  Else, the command will not be scheduled.
     * If no subsystems are required, return an empty set.
     *
     * <p>Note: it is recommended that user implementations contain the
     * requirements as a field, and return that field here, rather than allocating
     * a new set every time this is called.
     *
     * @return the set of subsystems that are required
     */
    const modm::LinkedList<Subsystem*>* getRequirements() const; //<- pointer stuff rough for now

    virtual void interrupted(void) = 0;

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
    void schedule() { schedule(true); }

    /**
     * Cancels this command.  Will call the command's interrupted() method.
     * Commands will be canceled even if they are not marked as interruptible.
     */
    void cancel();

    /**
     * Whether or not the command is currently scheduled.  Note that this does not
     * detect whether the command is being run by a CommandGroup, only whether it
     * is directly being run by the scheduler.
     *
     * @return Whether the command is scheduled.
     */
    bool isScheduled() const;

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
    bool hasRequirement(Subsystem* requirement) const;

    /**
     * Adds the required subsystem to a list of required subsystems
     */
    void addSubsystemRequirement(Subsystem* requirement);

    /**
     * Whether the given command should run when the robot is disabled.  Override
     * to return true if the command should run when disabled.
     *
     * @return whether the command should run when the robot is disabled
     */
    virtual bool runsWhenDisabled() const { return false; }

 private:
    bool isInterruptiable = true;

    modm::LinkedList<Subsystem*>* commandRequirements;
};

}  // namespace aruwlib

}  // namespace control

#endif
