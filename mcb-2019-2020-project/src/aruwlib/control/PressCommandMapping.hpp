#ifndef PRESS_COMMAND_MAPPING_HPP_
#define PRESS_COMMAND_MAPPING_HPP_

#include "CommandMapping.hpp"

namespace aruwlib
{
namespace control
{
class Command;
class RemoteMapState;

/**
 * A CommandMapping that adds `Command`s when the contained
 * mapping is a subset of the remote mapping. The Command is not removed
 * by the CommandMapping. Instead, the mapping will be removed when the
 * Command is finished.
 */
class PressCommandMapping : public CommandMapping
{
public:
    ///< Constructor must take the set of `Command`s and the RemoteMapState.
    PressCommandMapping(std::vector<Command *> cmds, const RemoteMapState &rms)
        : CommandMapping(cmds, rms),
          pressed(false)
    {
    }

    ///< Default destructor.
    ~PressCommandMapping() override = default;

    /**
     * See the class description details about how the commands are added.
     *
     * @param[in] currState The current RemoteMapState of the remote.
     */
    void executeCommandMapping(const RemoteMapState &currState) override;

private:
    bool pressed;
};  // class PressCommandMapping
}  // namespace control
}  // namespace aruwlib
#endif  // PRESS_COMMAND_MAPPING_HPP_
