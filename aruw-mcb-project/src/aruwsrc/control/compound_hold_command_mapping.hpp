/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef COMPOUND_HOLD_COMMAND_MAPPING_HPP_
#define COMPOUND_HOLD_COMMAND_MAPPING_HPP_

#include "tap/control/command_mapping.hpp"
#include "tap/drivers.hpp"
#include "tap/control/remote_map_state.hpp"

namespace aruwsrc::control
{

/**
 * Workaround for getting more complex switch state sets into command mappings.
 */
class CompoundHoldCommandMapping : public tap::control::CommandMapping
{
public:
    CompoundHoldCommandMapping(
        tap::Drivers *drivers,
        const std::vector<tap::control::Command *> cmds,
        const std::vector<const tap::control::RemoteMapState *> rmsVec)
        : tap::control::CommandMapping(drivers, cmds, *rmsVec[0]),
          rmsVec(rmsVec),
          commandScheduled(false)
    {
    }

    /**
     * Default destructor.
     */
    ~CompoundHoldCommandMapping() override = default;

    void executeCommandMapping(const tap::control::RemoteMapState &currState) override;

private:
    const std::vector<const tap::control::RemoteMapState *> rmsVec;
    bool commandScheduled;
};  // class CompoundHoldCommandMapping
}  // namespace aruwsrc::control

#endif  // TAPROOT_HOLD_COMMAND_MAPPING_HPP_
