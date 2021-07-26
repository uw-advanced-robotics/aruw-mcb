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

#include "xaxis_subsystem.hpp"

#include "aruwlib/drivers.hpp"

using tap::Drivers;

namespace aruwsrc
{
namespace engineer
{
void XAxisSubsystem::setExtended(bool isExtended)
{
    drivers->digital.set(pin, extended);
    extended = isExtended;
}

bool XAxisSubsystem::isExtended() const { return extended; }

void XAxisSubsystem::runHardwareTests()
{
    if (tap::arch::clock::getTimeMicroseconds() - testTime > 1000000)
        this->setHardwareTestsComplete();
}

void XAxisSubsystem::onHardwareTestStart()
{
    testTime = tap::arch::clock::getTimeMicroseconds();
    this->setExtended(!isExtended());
}

void XAxisSubsystem::onHardwareTestComplete() { this->setExtended(!isExtended()); }

}  // namespace engineer

}  // namespace aruwsrc
