#ifndef HOLD_COMMAND_MAPPING_HPP_
#define HOLD_COMMAND_MAPPING_HPP_

#include "CommandMapping.hpp"
namespace aruwlib
{
namespace control
{
class Command;
class RemoteMapState;

/**
 * A CommandMapping that adds `Command`s when the contained
 * mapping is a subset of the remote mapping and removes the `Command`s
 * when the mapping is no longer a subset.
 *
 * Additionally, When neg keys are being used and the mapping's neg keys
 * are a subset of the remote map state, the `Command`s are removed.
 */
class HoldCommandMapping : public CommandMapping
{
public:
    ///< Constructor must take the set of `Command`s and the RemoteMapState.
    HoldCommandMapping(std::vector<Command *> cmds, const RemoteMapState &rms)
        : CommandMapping(cmds, rms),
          commandScheduled(false)
    {
    }

    ///< Default destructor.
    ~HoldCommandMapping() override = default;

    /**
     * Adds `Command`s when `this`'s RemoteMapState is a subset of `currState`, removes
     * `Command`s when it is not. Also removes `Command`s when the neg keys of `this`'s
     * RemoteMapState is a subset of the keys in `currState`.
     *
     * @param[in] currState The current RemoteMapState of the remote.
     */
    void executeCommandMapping(const RemoteMapState &currState) override;

private:
    bool commandScheduled;
};  // class HoldCommandMapping
}  // namespace control
}  // namespace aruwlib

#endif  // HOLD_COMMAND_MAPPING_HPP_
