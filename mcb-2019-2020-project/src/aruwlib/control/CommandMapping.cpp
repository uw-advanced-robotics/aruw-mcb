#include "CommandMapping.hpp"

#include "aruwlib/Drivers.hpp"

#include "command.hpp"

#include <algorithm>

namespace aruwlib
{
namespace control
{
CommandMapping::CommandMapping(std::vector<Command *> cmds, const RemoteMapState &rms)
    : mapState(rms),
        mappedCommands(cmds)
{
    std::remove_if(cmds.begin(), cmds.end(), [](Command *c) { return c == nullptr; });
}

bool operator==(const CommandMapping &cm1, const CommandMapping &cm2)
{
    return (cm1.mapState == cm2.mapState) && (cm1.mappedCommands == cm2.mappedCommands);
}

bool mapStateEqual(const CommandMapping &cm1, const CommandMapping &cm2)
{
    // When inserting mappings into the CommandMapper, we want to check for equality based
    // on the mapState since we don't want two identical map_states with unique map_commands.
    // Even if mappedCommand vectors are different we want insertion to fail.
    return cm1.mapState == cm2.mapState;
}

bool CommandMapping::mappingSubset(const RemoteMapState &mapState)
{
    return this->mapState.stateSubset(mapState);
}

void CommandMapping::addCommands()
{
    for (Command *cmd : mappedCommands)
    {
        Drivers::commandScheduler.addCommand(cmd);
    }
}

void CommandMapping::removeCommands()
{
    for (Command *cmd : mappedCommands)
    {
        Drivers::commandScheduler.removeCommand(cmd, false);
    }
}
}  // namespace control
}  // namespace aruwlib