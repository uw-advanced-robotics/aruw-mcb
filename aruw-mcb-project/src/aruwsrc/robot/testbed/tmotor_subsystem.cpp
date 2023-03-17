/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifdef TARGET_TESTBED

#include "tmotor_subsystem.hpp"

#include <cassert>

namespace aruwsrc::testbed
{
TMotorSubsystem::TMotorSubsystem(tap::Drivers* drivers, aruwsrc::motor::Tmotor_AK809* tmotor)
    : tap::control::Subsystem(drivers),
      drivers(drivers),
      tmotor(tmotor),
      desiredOutput(0)
{
    assert(drivers != nullptr);
    assert(tmotor != nullptr);
}

void TMotorSubsystem::initialize()
{
    desiredOutput = 0;
    tmotor->initialize();
    tmotor->setDesiredOutput(0);
    tmotor->sendPositionHomeResetMessage();
}

void TMotorSubsystem::refresh() { tmotor->setDesiredOutput(desiredOutput); }

void TMotorSubsystem::onHardwareTestStart() { tmotor->setDesiredOutput(0); }
}  // namespace aruwsrc::testbed

#endif  // TARGET_TESTBED
