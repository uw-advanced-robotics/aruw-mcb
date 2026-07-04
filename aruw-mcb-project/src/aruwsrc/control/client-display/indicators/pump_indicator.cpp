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
    aruwsrc::control::digital::DigitalOutSubsystem,
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter)
    : HudIndicator(refSerialTransmitter),
      turretSubsystem(turretSubsystem)
{
}

modm::ResumableResult<void> PumpIndicator::update()
{
    RF_BEGIN(1);

    // Don't update the message if you're deleting it and it's already deleted
    if (prevOperation == Tx::GRAPHIC_DELETE &&
        damageGraphic.graphicData.operation == Tx::GRAPHIC_DELETE)
    {
        RF_RETURN();
    }

    RefSerialTransmitter::configLine(
        DAMAGE_INDICATOR_THICKNESS,
        X_POS + x,
        Y_POS + y,
        X_POS + x,
        Y_POS + LINE_LENGTH + y,
        &damageGraphic.graphicData);

    RF_CALL(refSerialTransmitter.sendGraphic(&damageGraphic));

    RF_END();
}

}  // namespace aruwsrc::control::client_display::indicators
