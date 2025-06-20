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

using Channel = tap::communication::serial::Remote::Channel;

namespace aruwsrc
{
namespace control::balstd
{

float BalstdControlOperatorInterface::getXVel() const
{
    return drivers->remote.getChannel(Channel::LEFT_VERTICAL) * MAX_X_VEL;
}

float BalstdControlOperatorInterface::getYawVel() const
{
    return -drivers->remote.getChannel(Channel::RIGHT_HORIZONTAL) * MAX_YAW_VEL;
}

float BalstdControlOperatorInterface::getHeightVel() const
{
    return drivers->remote.getChannel(Channel::WHEEL) * MAX_HEIGHT_VEL;
}

float BalstdControlOperatorInterface::getManualLegXForce() const
{
    return drivers->remote.getChannel(Channel::LEFT_HORIZONTAL) * MAX_LEG_FORCE;
}

float BalstdControlOperatorInterface::getManualLegYForce() const
{
    return -drivers->remote.getChannel(Channel::LEFT_VERTICAL) * MAX_LEG_FORCE;
}

float BalstdControlOperatorInterface::getManualWheelTorque() const
{
    return drivers->remote.getChannel(Channel::WHEEL) * MAX_WHEEL_TORQUE;
}

}  // namespace control::balstd
}  // namespace aruwsrc
