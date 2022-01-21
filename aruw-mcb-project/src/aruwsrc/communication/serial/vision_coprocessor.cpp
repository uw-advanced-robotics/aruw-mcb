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
#include "vision_coprocessor.hpp"

#include <cstring>

#include "aruwsrc/drivers.hpp"

using namespace tap::arch;
using namespace tap::serial;

namespace aruwsrc
{
namespace serial
{
VisionCoprocessor::VisionCoprocessor(aruwsrc::Drivers* drivers)
    : DJISerial(drivers, Uart::UartPort::Uart2),
      lastAimData(),
      aimDataValid(false),
      turretMCBCanComm(&drivers->turretMCBCanComm)
{
}

void VisionCoprocessor::messageReceiveCallback(const SerialMessage& completeMessage)
{
    switch (completeMessage.type)
    {
        case CV_MESSAGE_TYPE_TURRET_AIM:
        {
            if (decodeToTurretAimData(completeMessage, &lastAimData))
            {
                aimDataValid = true;
            }
            return;
        }
        default:
            return;
    }
}

bool VisionCoprocessor::decodeToTurretAimData(const SerialMessage& message, TurretAimData* aimData)
{
    if (message.length != sizeof(*aimData))
    {
        return false;
    }
    memcpy(aimData, &message.data, sizeof(*aimData));
    return true;
}

void VisionCoprocessor::sendMessage() { sendOdometryData(); }

void VisionCoprocessor::sendOdometryData()
{
    // TODO: add odometry data
    OdometryData odometryData;
    odometryData.chassisX = 0.0f;
    odometryData.chassisY = 0.0f;
    odometryData.chassisZ = 0.0f;
    odometryData.turretPitch = turretMCBCanComm->getPitch();
    odometryData.turretYaw = turretMCBCanComm->getYaw();
    odometryData.timestamp = tap::arch::clock::getTimeMicroseconds();
    memcpy(&txMessage.data, &odometryData, sizeof(odometryData));
    txMessage.type = CV_MESSAGE_ODOMETRY_DATA;
    txMessage.length = sizeof(odometryData);
    send();
}

}  // namespace serial
}  // namespace aruwsrc
