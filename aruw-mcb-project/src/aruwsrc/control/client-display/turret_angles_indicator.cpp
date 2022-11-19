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

#include "turret_angles_indicator.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/drivers.hpp"

using namespace tap::communication::serial;
using namespace tap::algorithms;

namespace aruwsrc::control::client_display
{
TurretAnglesIndicator::TurretAnglesIndicator(
    aruwsrc::Drivers &drivers,
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
    const aruwsrc::control::turret::RobotTurretSubsystem &robotTurretSubsystem)
    : HudIndicator(refSerialTransmitter),
      drivers(drivers),
      robotTurretSubsystem(robotTurretSubsystem)
{
}

modm::ResumableResult<bool> TurretAnglesIndicator::sendInitialGraphics()
{
    RF_BEGIN(0);

    // send turret angle data graphic and associated labels
    RF_CALL(refSerialTransmitter.sendGraphic(&turretAnglesGraphic));
    turretAnglesGraphic.graphicData.operation = Tx::GRAPHIC_MODIFY;
    RF_CALL(refSerialTransmitter.sendGraphic(&turretAnglesLabelGraphics));

    RF_END();
}

modm::ResumableResult<bool> TurretAnglesIndicator::update()
{
    RF_BEGIN(1);

    yaw = modm::toDegree(robotTurretSubsystem.getWorldYaw());
    pitch = modm::toDegree(robotTurretSubsystem.getWorldPitch());

    if (sendTurretDataTimer.execute() &&
        (!compareFloatClose(prevYaw, yaw, 1.0f / TURRET_ANGLES_DECIMAL_PRECISION) ||
         !compareFloatClose(prevPitch, pitch, 1.0f / TURRET_ANGLES_DECIMAL_PRECISION)))
    {
        updateTurretAnglesGraphicMsg();

        RF_CALL(refSerialTransmitter.sendGraphic(&turretAnglesGraphic));

        prevYaw = yaw;
        prevPitch = pitch;
    }

    RF_END();
}

void TurretAnglesIndicator::initialize()
{
    uint8_t turretAnglesName[3];
    getUnusedGraphicName(turretAnglesName);

    RefSerialTransmitter::configGraphicGenerics(
        &turretAnglesGraphic.graphicData,
        turretAnglesName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        TURRET_ANGLES_COLOR);

    RefSerialTransmitter::configCharacterMsg(
        TURRET_ANGLES_CHAR_SIZE,
        TURRET_ANGLES_CHAR_WIDTH,
        TURRET_ANGLES_START_X,
        TURRET_ANGLES_START_Y,
        "0\n\n0",
        &turretAnglesGraphic);

    turretAnglesName[2]++;

    RefSerialTransmitter::configGraphicGenerics(
        &turretAnglesLabelGraphics.graphicData,
        turretAnglesName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        TURRET_ANGLES_COLOR);

    RefSerialTransmitter::configCharacterMsg(
        TURRET_ANGLES_CHAR_SIZE,
        TURRET_ANGLES_CHAR_WIDTH,
        TURRET_ANGLES_START_X - strlen("PITCH: ") * TURRET_ANGLES_CHAR_SIZE,
        TURRET_ANGLES_START_Y,
        "  YAW:\n\nPITCH:",
        &turretAnglesLabelGraphics);

    prevYaw = 0.0f;
    prevPitch = 0.0f;
}

void TurretAnglesIndicator::updateTurretAnglesGraphicMsg()
{
    // set the character buffer `turretAnglesGraphic.msg` to the turret pitch/yaw angle
    // values
    // note that `%f` doesn't work in `snprintf` and neither do the integer or floating point
    // graphics, so this is why we are using `snprintf` to put floating point numbers in a
    // character graphic
    turretAnglesGraphic.msg[0] = yaw < 0 ? '-' : ' ';
    bytesWritten = 1;

    bytesWritten += snprintf(
        turretAnglesGraphic.msg + bytesWritten,
        sizeof(turretAnglesGraphic.msg) > bytesWritten
            ? sizeof(turretAnglesGraphic.msg) - bytesWritten
            : 0,
        "%i.%i\n\n",
        abs(static_cast<int>(yaw)),
        abs(static_cast<int>(yaw * TURRET_ANGLES_DECIMAL_PRECISION) %
            TURRET_ANGLES_DECIMAL_PRECISION));

    if (bytesWritten < sizeof(turretAnglesGraphic.msg))
    {
        turretAnglesGraphic.msg[bytesWritten] = (pitch < 0 ? '-' : ' ');
        bytesWritten++;
    }

    bytesWritten += snprintf(
        turretAnglesGraphic.msg + bytesWritten,
        sizeof(turretAnglesGraphic.msg) > bytesWritten
            ? sizeof(turretAnglesGraphic.msg) - bytesWritten
            : 0,
        "%i.%i\n\n",
        abs(static_cast<int>(pitch)),
        abs(static_cast<int>(pitch * TURRET_ANGLES_DECIMAL_PRECISION) %
            TURRET_ANGLES_DECIMAL_PRECISION));

    // because the RoboMaster client is trash, set all bytes after index bytesWritten to '\0'
    memset(
        turretAnglesGraphic.msg + bytesWritten,
        '\0',
        sizeof(turretAnglesGraphic.msg) > bytesWritten
            ? sizeof(turretAnglesGraphic.msg) - bytesWritten
            : 0);

    // `endAngle` is actually length of the string, but currently robomaster client doesn't actually
    // use it
    turretAnglesGraphic.graphicData.endAngle = bytesWritten;
}

}  // namespace aruwsrc::control::client_display
