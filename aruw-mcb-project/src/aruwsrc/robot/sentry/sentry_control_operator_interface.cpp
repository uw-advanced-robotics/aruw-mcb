/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "sentry_control_operator_interface.hpp"
#include "tap/architecture/clock.hpp"

namespace aruwsrc::control::sentry
{

float SentryControlOperatorInterface::getChassisXVelocity()
{
    
}


uint32_t getDT(uitn32_t prevTimeCalled) {
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime tap::arch::clock::getTimeMilliseconds();
    return currTime - prevTimeCalled;
}

}
