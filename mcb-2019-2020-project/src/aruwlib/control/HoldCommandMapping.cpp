#include "HoldCommandMapping.hpp"

namespace aruwlib
{
namespace control
{
void HoldCommandMapping::executeCommandMapping(const RemoteMapState &currState)
{
    if (mappingSubset(currState) &&
        (!mapState.getNegKeysUsed() ||
         ((mapState.getNegKeys() & currState.getKeys()) != mapState.getNegKeys())))
    {
        if (!commandScheduled)
        {
            commandScheduled = true;
            addCommands();
        }
    }
    else if (commandScheduled)
    {
        commandScheduled = false;
        removeCommands();
    }
}
}  // namespace control
}  // namespace aruwlib
