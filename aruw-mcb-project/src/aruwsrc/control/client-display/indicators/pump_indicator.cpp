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

#include "pump_indicator.hpp"

#include "tap/architecture/clock.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display::indicators
{
PumpIndicator::PumpIndicator(
    tap::communication::serial::RefSerialTransmitter& refSerialTransmitter,
    aruwsrc::control::digital::DigitalOutSubsystem& subsystem)
    : HudIndicator(refSerialTransmitter),
      subsystem(subsystem)
{
}

modm::ResumableResult<void> PumpIndicator::update()
{
    uint32_t prevOperation = -1;

    RF_BEGIN(1);

    prevOperation = pumpGraphic.graphicData.operation;

    if (subsystem.getState())
    {
        pumpGraphic.graphicData.operation =
            prevOperation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD : Tx::GRAPHIC_MODIFY;
    }
    else
    {
        pumpGraphic.graphicData.operation = Tx::GRAPHIC_DELETE;
    }

    // Don't resend if it's already deleted and staying deleted
    if (prevOperation == Tx::GRAPHIC_DELETE &&
        pumpGraphic.graphicData.operation == Tx::GRAPHIC_DELETE)
    {
        RF_RETURN();
    }

    RefSerialTransmitter::configCircle(
        PUMP_INDICATOR_RADIUS,
        X_POS,
        Y_POS,
        PUMP_INDICATOR_RADIUS,
        &pumpGraphic.graphicData);

    RF_CALL(refSerialTransmitter.sendGraphic(&pumpGraphic));

    RF_END();
}

void PumpIndicator::initialize()
{
    uint8_t indicatorName[3];

    getUnusedGraphicName(indicatorName);
    RefSerialTransmitter::configGraphicGenerics(
        &pumpGraphic.graphicData,
        indicatorName,
        Tx::GRAPHIC_DELETE,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::GREEN);  // pick whichever color distinguishes this from other indicators
}

}  // namespace aruwsrc::control::client_display::indicators