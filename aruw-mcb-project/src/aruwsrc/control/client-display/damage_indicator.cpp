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
    float prevPeakAngle = peakAngleBin.radians.getWrappedValue();
    uint32_t prevOperation = -1;

    RF_BEGIN(1);

    peakAngleBin = plateHitTracker.getPeakAnglesRadians()[0];

    // Check if current angle is different from previous angle
    if (peakAngleBin.radians.getWrappedValue() != prevPeakAngle)
    {
        decayTimeout.restart(DECAY_TIMEOUT_MILLIS);
    }

    // Get position of hit in turret frame + offset
    hitAngleRadian = peakAngleBin.radians.getWrappedValue() - turretSubsystem.getWorldYaw();

    // Normalize angle to be between 0 and 2pi
    hitAngleRadian = fmod(hitAngleRadian, 2 * static_cast<float>(M_PI));
    if (hitAngleRadian < 0)
    {
        hitAngleRadian += 2 * static_cast<float>(M_PI);
    }

    prevOperation = damageGraphic.graphicData.operation;

    // If the damage is old or the angle is within +-45 deg
    if (decayTimeout.isExpired() || decayTimeout.isStopped() ||
        (fmod(hitAngleRadian + CENTER_THRESHOLD, 2 * static_cast<float>(M_PI)) <
         CENTER_THRESHOLD * 2))
    {
        damageGraphic.graphicData.operation = Tx::GRAPHIC_DELETE;
    }
    else
    {
        damageGraphic.graphicData.operation =
            prevOperation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD : Tx::GRAPHIC_MODIFY;
    }

    // Add in offset, because 0 should be top
    hitAngleRadian += INDICATOR_OFFSET_RADIANS;

    // Calculate x and y position of hit
    x = cos(hitAngleRadian) * DISTANCE_FROM_CENTER;
    y = sin(hitAngleRadian) * DISTANCE_FROM_CENTER;

    // Don't update the message if you're deleting it and it's already deleted
    if (prevOperation == Tx::GRAPHIC_DELETE &&
        damageGraphic.graphicData.operation == Tx::GRAPHIC_DELETE)
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

    RF_END_RETURN(true);
}

modm::ResumableResult<bool> DamageIndicator::sendInitialGraphics()
{
    RF_BEGIN(0);
    RF_END_RETURN(true);
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
