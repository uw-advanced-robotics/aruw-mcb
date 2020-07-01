#ifndef HOLD_REPEAT_MAPPING_HPP_
#define HOLD_REPEAT_MAPPING_HPP_

#include "CommandMapping.hpp"
namespace aruwlib
{
namespace control
{
class Command;
class RemoteMapState;

/**
 * A CommandMapping that adds `Command`s when the contained
 * mapping is a subset of the remote mapping. If a Command finishes and the
 * contained mapping is still a subset of the remote mapping, it is added again.
 * It then removes the `Command`s when the mapping is no longer a subset.
 *
 * Additionally, When neg keys are being used and the mapping's neg keys
 * are a subset of the remote map state, the `Command`s are removed.
 */
class HoldRepeatCommandMapping : public CommandMapping
{
public:
    ///< Constructor must take the set of `Command`s and the RemoteMapState.
    HoldRepeatCommandMapping(std::vector<Command *> cmds, const RemoteMapState &rms)
        : CommandMapping(cmds, rms)
    {
    }

    ///< Default destructor.
    ~HoldRepeatCommandMapping() override = default;

    /**
     * See the class description details about how the commands are added and
     * removed.
     *
     * @param[in] currState The current RemoteMapState of the remote.
     */
    void executeCommandMapping(const RemoteMapState &currState) override;
};  // class HoldCommandMapping
}  // namespace control
}  // namespace aruwlib

#endif  // HOLD_REPEAT_COMMAND_MAPPING_HPP_
