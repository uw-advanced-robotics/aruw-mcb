/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "balstd_control_operator_interface.hpp"

namespace aruwsrc::balstd
{
float BalstdControlOperatorInterface::getRemoteChannel(Channel channel)
{
    float val = drivers->remote.getChannel(channel);
    if (tap::algorithms::compareFloatClose(val, 0, 1e-3))
    {
        channelHeldOver[static_cast<size_t>(channel)] = false;
    }
    return channelHeldOver[static_cast<size_t>(channel)] ? 0.0f : val;
}

float BalstdControlOperatorInterface::getModeRestrictedInput(Channel channel, Mode mode, float max)
{
    return (this->mode == mode) ? getRemoteChannel(channel) * max : 0.0f;
}

}  // namespace aruwsrc::balstd
