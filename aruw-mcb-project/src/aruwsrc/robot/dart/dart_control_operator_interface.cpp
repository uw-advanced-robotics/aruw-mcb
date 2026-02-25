/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include "aruwsrc/robot/dart/dart_control_operator_interface.hpp"

namespace aruwsrc::control::dart
{
float DartControlOperatorInterface::getPullbackVelocity()
{
    // TODO: CHECK IF THIS IS THE CORRECT DIRECTION
    return drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
}

float DartControlOperatorInterface::getYawVelocity()
{
    return drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);
}
}  // namespace aruwsrc::control::dart