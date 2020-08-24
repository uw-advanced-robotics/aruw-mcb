#include "CommandMapper.hpp"

#include "aruwlib/errors/create_errors.hpp"

#include "HoldCommandMapping.hpp"
#include "HoldRepeatCommandMapping.hpp"
#include "PressCommandMapping.hpp"
#include "RemoteMapState.hpp"
#include "ToggleCommandMapping.hpp"

using namespace aruwlib::errors;

namespace aruwlib
{
namespace control
{
CommandMapper::~CommandMapper()
{
    for (CommandMapping *cmdMap : commandsToRun)
    {
        delete cmdMap;
    }
}

void CommandMapper::handleKeyStateChange(
    uint16_t key,
    Remote::SwitchState leftSwitch,
    Remote::SwitchState rightSwitch,
    bool mouseL,
    bool mouseR)
{
    // Make a new map state that represents the current state of the remote,
    // to be passed in to each of the CommandMappings.
    RemoteMapState mapstate;
    mapstate.initLSwitch(leftSwitch);
    mapstate.initRSwitch(rightSwitch);
    mapstate.initKeys(key);
    if (mouseL)
    {
        mapstate.initLMouseButton();
    }
    if (mouseR)
    {
        mapstate.initRMouseButton();
    }

    for (CommandMapping *cmdMap : commandsToRun)
    {
        cmdMap->executeCommandMapping(mapstate);
    }
}

void CommandMapper::addMap(CommandMapping *mapping)
{
    for (const CommandMapping *const cmap : commandsToRun)
    {
        if (mapStateEqual(*cmap, *mapping))
        {
            RAISE_ERROR(
                "failed to insert io mapping",
                aruwlib::errors::CONTROLLER_MAPPER,
                aruwlib::errors::INVALID_ADD);
            return;
        }
    }
    commandsToRun.push_back(mapping);
}

void CommandMapper::addHoldMapping(
    const RemoteMapState &mapping,
    const std::vector<Command *> commands)
{
    addMap(new HoldCommandMapping(commands, mapping));
}

void CommandMapper::addHoldRepeatMapping(
    const RemoteMapState &mapping,
    std::vector<Command *> commands)
{
    addMap(new HoldRepeatCommandMapping(commands, mapping));
}

void CommandMapper::addToggleMapping(
    const RemoteMapState &mapping,
    const std::vector<Command *> commands)
{
    addMap(new ToggleCommandMapping(commands, mapping));
}

void CommandMapper::addPressMapping(
    const RemoteMapState &mapping,
    const std::vector<Command *> commands)
{
    addMap(new PressCommandMapping(commands, mapping));
}

}  // namespace control
}  // namespace aruwlib
