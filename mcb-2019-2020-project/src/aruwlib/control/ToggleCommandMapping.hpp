#ifndef TOGGLE_COMMAND_MAPPING_HPP_
#define TOGGLE_COMMAND_MAPPING_HPP_

#include "CommandMapping.hpp"

namespace aruwlib
{
namespace control
{
class Command;
class RemoteMapState;

/**
 * A CommandMapping that adds `Command`s when the contained mapping
 * is toggled, and removes the `Command`s when the mapping is untoggled.
 */
class ToggleCommandMapping : public CommandMapping
{
public:
    ///< Constructor must take the set of `Command`s and the RemoteMapState.
    ToggleCommandMapping(std::vector<Command *> cmds, const RemoteMapState &rms)
        : CommandMapping(cmds, rms),
          pressed(false),
          toggled(false)
    {
    }

    ///< Default destructor.
    ~ToggleCommandMapping() override = default;

    /**
     * See the class description details about how the commands are added and
     * removed.
     *
     * @param[in] currState The current RemoteMapState of the remote.
     */
    void executeCommandMapping(const RemoteMapState &currState) override;

private:
    bool pressed;
    bool toggled;
};  // class PressCommandMapping
}  // namespace control
}  // namespace aruwlib

#endif  //  TOGGLE_COMMAND_MAPPING_HPP_
