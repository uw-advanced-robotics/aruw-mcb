#include "ToggleCommandMapping.hpp"

namespace aruwlib
{
namespace control
{
void ToggleCommandMapping::executeCommandMapping(const RemoteMapState &currState)
{
    // Neg keys are weird in this mapping and must be handled as such. If neg keys of the
    // currState equal the mapping's state, the mapping must be reset and commands removed.
    if (mapState.getNegKeysUsed() &&
        ((mapState.getNegKeys() & currState.getKeys()) == mapState.getNegKeys()))
    {
        toggled = false;
        pressed = false;
        removeCommands();
    }
    else if (mappingSubset(currState))
    {
        if (!pressed)
        {
            if (!toggled)
            {
                addCommands();
            }
            else
            {
                removeCommands();
            }
            toggled = !toggled;
            pressed = true;
        }
    }
    else
    {
        pressed = false;
    }
}
}  // namespace control
}  // namespace aruwlib
