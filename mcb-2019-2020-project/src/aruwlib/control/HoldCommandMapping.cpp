#include "HoldCommandMapping.hpp"

namespace aruwlib
{
namespace control
{
void HoldCommandMapping::executeCommandMapping(const RemoteMapState &currState)
{
    if (mappingSubset(currState) &&
        (!mapState.getNegKeysUsed() || !negKeysSubset(mapState, currState)))
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
