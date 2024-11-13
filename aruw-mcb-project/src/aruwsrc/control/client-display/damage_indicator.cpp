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

#include "damage_indicator.hpp"

#include "tap/architecture/clock.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
DamageIndicator::DamageIndicator(
    aruwsrc::algorithms::PlateHitTracker &plateHitTracker,
    const aruwsrc::control::turret::RobotTurretSubsystem &turretSubsystem,
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter)
    : HudIndicator(refSerialTransmitter),
      plateHitTracker(plateHitTracker),
      turretSubsystem(turretSubsystem)
{
}

modm::ResumableResult<bool> DamageIndicator::update()
{
    float hitAngleRadian = 0;
    uint32_t prevOperation = -1;
    bool angleIsClose = false;

    RF_BEGIN(1);

    // Check if current angle is different from previous angle
    if (plateHitTracker.getPeakAnglesRadians()[0].radians.getWrappedValue() !=
        peakAngleBin.radians.getWrappedValue())
    {
        decayTimeout.restart(DECAY_TIMEOUT_MILLIS);
    }
    peakAngleBin = plateHitTracker.getPeakAnglesRadians()[0];

    // Get position of hit in turret frame + offset
    hitAngleRadian = peakAngleBin.radians.getWrappedValue();
    hitAngleRadian += -turretSubsystem.getWorldYaw();
    hitAngleRadian += INDICATOR_OFFSET_RADIANS;

    // Check if the angle is close to the previous angle
    angleIsClose = anglesAreClose(hitAngleRadian, prevComputedAngle);
    prevComputedAngle = hitAngleRadian;

    prevOperation = damageGraphic.graphicData.operation;

    if (!decayTimeout.isExpired())
    {
        damageGraphic.graphicData.operation =
            prevOperation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD : Tx::GRAPHIC_MODIFY;
    }
    else
    {
        damageGraphic.graphicData.operation = Tx::GRAPHIC_DELETE;
    }

    // Calculate x and y position of hit
    x = cos(hitAngleRadian) * DISTANCE_FROM_CENTER;
    y = sin(hitAngleRadian) * DISTANCE_FROM_CENTER;

    // If the angles aren't different and the operation is the same, don't send the graphic
    if (angleIsClose && prevOperation == damageGraphic.graphicData.operation)
    {
        RF_RETURN(true);
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

modm::ResumableResult<bool> DamageIndicator::sendInitialGraphics()
{
    RF_BEGIN(0);
    RF_END();
}

void DamageIndicator::initialize()
{
    uint8_t indicatorName[3];

    getUnusedGraphicName(indicatorName);
    RefSerialTransmitter::configGraphicGenerics(
        &damageGraphic.graphicData,
        indicatorName,
        Tx::GRAPHIC_DELETE,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::PURPLISH_RED);
}

}  // namespace aruwsrc::control::client_display
