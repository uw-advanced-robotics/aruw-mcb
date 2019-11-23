#ifndef __COMMAND_GROUP_HPP__
#define __COMMAND_GROUP_HPP__

#include <modm/container/dynamic_array.hpp>
#include "command.hpp"

namespace aruwlib
{

namespace control
{

/**
 * A CommandGroup is a list of commands which are executed in sequence.
 * The command group stores the list of commands in the format of a
 * finite state machine.
 *
 * @see Command
 * @see Subsystem
 */
class CommandGroup : public Command
{
    CommandGroup() = default;

    ~CommandGroup() = default;

    CommandGroup& operator=(CommandGroup&&) = default;

    /**
     * Adds a new Command to the group. The Command will be started after all the
     * previously added Commands.
     *
     * Note that any requirements the given Command has will be added to the
     * group. For this reason, a Command's requirements can not be changed after
     * being added to a group.
     *
     * It is recommended that this method be called in the constructor.
     *
     * @param command The Command to be added
     */
    void AddSequential(Command* command);

    /**
     * Adds a new child Command to the group. The Command will be started after
     * all the previously added Commands.
     *
     * Instead of waiting for the child to finish, a CommandGroup will have it run
     * at the same time as the subsequent Commands. The child will run until
     * either it finishes, a new child with conflicting requirements is started,
     * or the main sequence runs a Command with conflicting requirements.
     * 
     * Note that any requirements the given Command has will be added to the
     * group. For this reason, a Command's requirements can not be changed after
     * being added to a group.
     *
     * It is recommended that this method be called in the constructor.
     *
     * @param command The command to be added
     */
    void AddParallel(Command* command);

    int getSize() const;

    virtual void Initialize(void) = 0;

    virtual void execute(void) = 0;

    virtual bool isFinished(void) = 0;

    virtual void end(bool interrupted) = 0;

 private:
    int currentCommandIndex = -1;
};

}  // namespace control

}  // namespace aruwlib

#endif
