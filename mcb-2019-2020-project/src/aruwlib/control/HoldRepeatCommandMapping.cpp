#include "HoldRepeatCommandMapping.hpp"

#include "aruwlib/Drivers.hpp"

namespace aruwlib
{
namespace control
{
void HoldRepeatCommandMapping::executeCommandMapping(const RemoteMapState &currState)
{
    if (mappingSubset(currState) &&
        (!mapState.getNegKeysUsed() ||
         ((mapState.getNegKeys() & currState.getKeys()) != mapState.getNegKeys())))
    {
        for (Command *cmd : mappedCommands)
        {
            if (!Drivers::commandScheduler.isCommandScheduled(cmd))
            {
                Drivers::commandScheduler.addCommand(cmd);
            }
        }
    }
    else
    {
        removeCommands();
    }
}
}  // namespace control
}  // namespace aruwlib
