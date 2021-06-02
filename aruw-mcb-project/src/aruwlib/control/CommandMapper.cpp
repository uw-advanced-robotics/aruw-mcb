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

#include "aruwlib/Drivers.hpp"
#include "aruwlib/errors/create_errors.hpp"

#include "CommandMapping.hpp"
#include "RemoteMapState.hpp"

using namespace aruwlib::errors;

namespace aruwlib
{
namespace control
{
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
                drivers,
                "failed to insert io mapping",
                aruwlib::errors::CONTROLLER_MAPPER,
                aruwlib::errors::ControllerMapperErrorType::INVALID_ADD);
            return;
        }
    }
    commandsToRun.push_back(mapping);
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
