#include "PressCommandMapping.hpp"

namespace aruwlib
{
namespace control
{
void PressCommandMapping::executeCommandMapping(const RemoteMapState &currState)
{
    if (mappingSubset(currState) &&
        (!mapState.getNegKeysUsed() || !negKeysSubset(mapState, currState)))
    {
        if (!pressed)
        {
            pressed = true;
            addCommands();
        }
    }
    else
    {
        pressed = false;
    }
}
}  // namespace control
}  // namespace aruwlib