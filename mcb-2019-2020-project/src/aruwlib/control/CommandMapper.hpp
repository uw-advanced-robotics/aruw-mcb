#ifndef COMMAND_MAPPER_HPP_
#define COMMAND_MAPPER_HPP_

#include <vector>

#include "aruwlib/communication/remote.hpp"

#include "CommandMapping.hpp"

namespace aruwlib
{
namespace control
{
/**
 * Control for mapping commands to actions. All the remote
 * mappings will be handled here. One passes a RemoteMapState and a set
 * of `Command`s for which the RemoteMapState is mapped to to one of
 * the `add<type>Mapping` functions. This will create an appropriate CommandMapping
 * and add it to the list of mappings to be checked each time new remote information
 * is received.
 *
 * For example, given the command `coolCommand`, to map a hold mapping
 * to the left switch in the up position, we call
 * `addHoldMapping(RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP),
 * {&coolCommand});`
 *
 * @note Only unique RemoteMapStates can be added to the CommandMapper. This insures
 *      a user will not accidently map two `Command`s to the same RemoteMapState without
 *      knowing they did so. Instead, the user must explicitly add `Command`s to a common
 *      list that maps to a single RemoteMapState.
 */
class CommandMapper
{
public:
    /**
     * Default construction.
     */
    CommandMapper() = default;

    /**
     * `delete`s the `CommandMapping`s that are in `commandsToRun`.
     */
    ~CommandMapper();

    /**
     * The heart of the CommandMapper.
     *
     * Iterates through all the current mappings to see which buttons are pressed
     * in order to determine which commands should be added to or removed from the scheduler.
     */
    void handleKeyStateChange(
        uint16_t key,
        Remote::SwitchState leftSwitch,
        Remote::SwitchState rightSwitch,
        bool mouseL,
        bool mouseR);

    /**
     * Attaches a Command to a remote control mapping which is added to the
     * CommandScheduler once when the mapping is satisfied and is removed
     * when the mapping is stopped being satisfied.
     *
     * @param[in] mapping a particular remote mapping associated with the Command.
     * @param[in] Command the Command to be triggered by this mapping.
     */
    void addHoldMapping(const RemoteMapState &mapping, const std::vector<Command *> commands);

    /**
     * Attaches a Command to a remote control mapping which is added to
     * the CommandScheduler when the remote state matches the passed in
     * mapping, starting the Command over while the mapping is still met
     * if it ever finishes.
     *
     * @param[in] mapping a particular remote mapping associated with the Command.
     * @param[in] Command the Command to be triggered by this mapping.
     */
    void addHoldRepeatMapping(const RemoteMapState &mapping, const std::vector<Command *> commands);

    /**
     * Attaches a Command to a remote control mapping which adds the command
     * to the CommandScheduler whenever a mapping is toggled.
     *
     * @param[in] mapping a particular remote mapping associated with the Command.
     * @param[in] Command the Command to be triggered by this mapping.
     * @note a toggle mapping is interrupted when the key is untoggled.
     */
    void addToggleMapping(const RemoteMapState &mapping, const std::vector<Command *> commands);

    /**
     * Attaches a Command to a remote control mapping which is added to the
     * CommandScheduler once each time the mapping is satisfied.
     *
     * @param[in] mapping a particular remote mapping associated with the Command.
     * @param[in] Command the Command to be triggered by this mapping.
     */
    void addPressMapping(const RemoteMapState &mapping, const std::vector<Command *> commands);

    /**
     * @return the number of command mappings in the mapper.
     */
    int getSize() const { return commandsToRun.size(); }

private:
    friend class CommandMapperFormatGenerator;
    /**
     * A helper function that verifies the mapping passed in can be added to `commandsToRun`
     * and if possible adds the mapping.
     */
    void addMap(CommandMapping *mapping);

    /**
     * We use a vector because it is slightly faster for iteration than an `std::set`. While
     * inserting, we insure CommandMappings with identical RemoteMapStates fail to be added.
     * It ends up being slow to insert, but this is OK since we only insert at the beginning
     * of the program.
     */
    std::vector<CommandMapping *> commandsToRun;
};

}  // namespace control
}  // namespace aruwlib

#endif  // COMMAND_MAPPER_HPP_
