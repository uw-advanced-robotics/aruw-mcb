#include "PressCommandMapping.hpp"

namespace aruwlib
{
namespace control
{
void PressCommandMapping::executeCommandMapping(const RemoteMapState &currState)
{
    if (mappingSubset(currState) &&
        (!this->mapState.getNegKeysUsed() ||
         ((mapState.getNegKeys() & currState.getKeys()) != mapState.getNegKeys())))
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