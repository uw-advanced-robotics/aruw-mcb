#ifndef COMMAND_MAPPING_HPP_
#define COMMAND_MAPPING_HPP_

#include <vector>

#include "aruwlib/control/command_scheduler.hpp"

#include "RemoteMapState.hpp"

namespace aruwlib
{
namespace control
{
class Command;
/**
 * A class that combines a vector of `Command`s and a RemoteMapState whose behavior
 * is defined by derived classes. Used in conjunction with the CommandMapper to
 * add and remove `Command`s from the scheduler when the derived CommandMapping
 * sees fit, depending on the current state of the remote data.
 *
 * @see HoldCommandMapping
 * @see HoldRepeatCommandMapping
 * @see PressCommandMapping
 * @see ToggleCommandMapping
 */
class CommandMapping
{
public:
    /**
     * Initializes the CommandMapping with the set of passed in `Command`s mapped to
     * a particular RemoteMapState.
     * 
     * @note All nullptr `Command`s in cmds will be removed.
     * @param[in] cmds A list of `Command`s that are associated with the mapping.
     * @param[in] rms The map state that will be compared to the actual remote state
     *      to determine whether or not to add `cmds`.
     */
    CommandMapping(std::vector<Command *> cmds, const RemoteMapState &rms);

    ///< Copy construction disallowed.
    CommandMapping(const CommandMapping &) = delete;

    ///< Copying disallowed.
    CommandMapping &operator=(const CommandMapping &) = delete;

    ///< Straight equality of the mapState and mappedCommands between cm1 and cm2.
    friend bool operator==(const CommandMapping &cm1, const CommandMapping &cm2);

    ///< Checks for equality between the `mapState`s of cm1 and cm2.
    friend bool mapStateEqual(const CommandMapping &cm1, const CommandMapping &cm2);

    ///< Nothing dynamically allocated that isn't taken care of automatically.
    virtual ~CommandMapping() = default;

    /**
     * Using currState, determines whether or not to add or remove `Command`s from
     * the main scheduler. Up the the implementer to determine what the criteria
     * for adding and removing `Command`s should be.
     *
     * @param[in] currState The current state of the remote.
     */
    virtual void executeCommandMapping(const RemoteMapState &currState) = 0;

    /**
     * @return `true` if `this`'s `mapState` is a subset of the pasesed in
     *      `mapState`. Returns `false` otherwise.
     */
    virtual bool mappingSubset(const RemoteMapState &mapState);

protected:
    ///< The RemoteMapState specified when constructing the CommandMapping.
    const RemoteMapState mapState;

    ///< A map of commands to add to and remove from the scheduler.
    std::vector<Command *> mappedCommands;

    ///< Adds all the `Command`s to the main CommandScheduler.
    void addCommands();

    ///< Removes all the `Command`s from the main CommandScheduler.
    void removeCommands();

    friend class CommandMapperFormatGenerator;
};  // class CommandMapping
}  // namespace control
}  // namespace aruwlib

#endif  // COMMAND_MAPPING_HPP_
