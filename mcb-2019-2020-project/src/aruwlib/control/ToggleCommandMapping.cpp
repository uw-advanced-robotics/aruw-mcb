#include "ToggleCommandMapping.hpp"

namespace aruwlib
{
namespace control
{
void ToggleCommandMapping::executeCommandMapping(const RemoteMapState &currState)
{
    // Neg keys are weird in this mapping and must be handled as such. If neg keys of the
    // map state are a subset of the currState's neg keys, the mapping must be reset
    // and commands removed.
    if (mapState.getNegKeysUsed() && negKeysSubset(mapState, currState))
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
