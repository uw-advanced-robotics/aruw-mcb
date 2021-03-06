/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

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
            delete mapping;
            RAISE_ERROR(
                drivers,
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
    addMap(new HoldCommandMapping(drivers, commands, mapping));
}

void CommandMapper::addHoldRepeatMapping(
    const RemoteMapState &mapping,
    std::vector<Command *> commands)
{
    addMap(new HoldRepeatCommandMapping(drivers, commands, mapping));
}

void CommandMapper::addToggleMapping(
    const RemoteMapState &mapping,
    const std::vector<Command *> commands)
{
    addMap(new ToggleCommandMapping(drivers, commands, mapping));
}

void CommandMapper::addPressMapping(
    const RemoteMapState &mapping,
    const std::vector<Command *> commands)
{
    addMap(new PressCommandMapping(drivers, commands, mapping));
}

const CommandMapping *CommandMapper::getAtIndex(std::size_t index) const
{
    if (index >= commandsToRun.size())
    {
        return nullptr;
    }
    return commandsToRun[index];
}
}  // namespace control
}  // namespace aruwlib
