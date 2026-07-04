/*
 * Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "pump_indicator.hpp"

#include "tap/architecture/clock.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display::indicators
{

PumpIndicator::PumpIndicator(
    aruwsrc::algorithms::PlateHitTracker& plateHitTracker,
    tap::communication::serial::RefSerialTransmitter& refSerialTransmitter,
    aruwsrc::control::digital::DigitalOutSubsystem& subsystem)
    : HudIndicator(refSerialTransmitter),
      plateHitTracker(plateHitTracker),
      subsystem(subsystem)
{
}

modm::ResumableResult<void> PumpIndicator::update()
{
    uint32_t prevOperation = -1;
    RF_BEGIN(1);

    if ()
        RefSerialTransmitter::configCircle(
            PUMP_INDICATOR_THICKNESS,
            X_POS,
            Y_POS,
            PUMP_INDICATOR_THICKNESS,
            &damageGraphic.graphicData);

    RF_END();
    prevOperation = damageGraphic.graphicData.operation;
}

}  // namespace aruwsrc::control::client_display::indicators
