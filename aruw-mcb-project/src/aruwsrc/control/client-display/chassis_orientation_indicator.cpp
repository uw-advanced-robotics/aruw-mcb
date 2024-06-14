/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "chassis_orientation_indicator.hpp"

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/drivers.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
ChassisOrientationIndicator::ChassisOrientationIndicator(
    tap::Drivers &drivers,
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
    const aruwsrc::control::turret::RobotTurretSubsystem &turretSubsystem,
    const std::vector<tap::control::Command *> avoidanceCommands)
    : HudIndicator(refSerialTransmitter),
      drivers(drivers),
      turretSubsystem(turretSubsystem),
      avoidanceCommands(avoidanceCommands)
{
}

modm::ResumableResult<bool> ChassisOrientationIndicator::sendInitialGraphics()
{
    RF_BEGIN(0)

    // send initial chassis orientation graphics
    RF_CALL(refSerialTransmitter.sendGraphic(&chassisOrientationGraphics));
    chassisOrientationGraphics.graphicData[0].operation = Tx::GRAPHIC_MODIFY;
    chassisOrientationGraphics.graphicData[1].operation = Tx::GRAPHIC_MODIFY;

    RF_END();
}

modm::ResumableResult<bool> ChassisOrientationIndicator::update()
{
    bool avoidance = false, modified = false;
    // This needs to be outside RF because of the loop variable
    for (auto currentCommand: this->avoidanceCommands)
    {
        avoidance |= drivers.commandScheduler.isCommandScheduled(currentCommand);
    }

    RF_BEGIN(1);
    // update chassisOrientation if turret is online
    // otherwise don't rotate chassis
    chassisOrientation.rotate(
        turretSubsystem.yawMotor.isOnline() ? -turretSubsystem.yawMotor.getAngleFromCenter()
                                            : 0.0f);

    // if chassis orientation has changed, send new graphic with updated orientation
    if (chassisOrientation != chassisOrientationPrev)
    {
        // since chassisOrientation is a pixel coordinate centered around
        // `CHASSIS_CENTER_X/Y`, center the line about these coordinates during configuration
        RefSerialTransmitter::configLine(
            CHASSIS_WIDTH,
            CHASSIS_CENTER_X + chassisOrientation.x,
            CHASSIS_CENTER_Y + chassisOrientation.y,
            CHASSIS_CENTER_X - chassisOrientation.x,
            CHASSIS_CENTER_Y - chassisOrientation.y,
            &chassisOrientationGraphics.graphicData[0]);
        modified = true;

        chassisOrientationPrev = chassisOrientation;
    }

    modified |= this->prevAvoidance != avoidance;
    this->prevAvoidance = avoidance;

    if (modified)
    {
        chassisOrientationGraphics.graphicData->color = static_cast<uint8_t>(
            avoidance ? CHASSIS_ORIENTATION_AVOIDANCE_COLOR : CHASSIS_ORIENTATION_STILL_COLOR);
        
        RF_CALL(refSerialTransmitter.sendGraphic(&chassisOrientationGraphics));
    }
    // reset rotated orientation back to forward orientation so next time chassisOrientation
    // is rotated by `getYawAngleFromCenter` the rotation is relative to the forward.
    chassisOrientation.set(0, CHASSIS_LENGTH / 2);

    RF_END();
}

void ChassisOrientationIndicator::initialize()
{
    // chassis orientation starts forward facing
    chassisOrientation.set(0, CHASSIS_LENGTH / 2);
    chassisOrientationPrev = chassisOrientation;

    uint8_t chassisOrientationName[3];
    getUnusedGraphicName(chassisOrientationName);

    // config the chassis graphic

    RefSerialTransmitter::configGraphicGenerics(
        &chassisOrientationGraphics.graphicData[0],
        chassisOrientationName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        CHASSIS_ORIENTATION_STILL_COLOR);

    RefSerialTransmitter::configLine(
        CHASSIS_WIDTH,
        CHASSIS_CENTER_X + chassisOrientation.x,
        CHASSIS_CENTER_Y + chassisOrientation.y,
        CHASSIS_CENTER_X - chassisOrientation.x,
        CHASSIS_CENTER_Y - chassisOrientation.y,
        &chassisOrientationGraphics.graphicData[0]);

    getUnusedGraphicName(chassisOrientationName);

    // config the turret graphic

    RefSerialTransmitter::configGraphicGenerics(
        &chassisOrientationGraphics.graphicData[1],
        chassisOrientationName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        CHASSIS_BARREL_COLOR);

    RefSerialTransmitter::configLine(
        CHASSIS_BARREL_WIDTH,
        CHASSIS_CENTER_X,
        CHASSIS_CENTER_Y,
        CHASSIS_CENTER_X,
        CHASSIS_CENTER_Y + CHASSIS_BARREL_LENGTH,
        &chassisOrientationGraphics.graphicData[1]);
}
}  // namespace aruwsrc::control::client_display
