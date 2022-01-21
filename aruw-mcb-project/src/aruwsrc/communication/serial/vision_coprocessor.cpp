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

#include "tap/architecture/endianness_wrappers.hpp"

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

bool VisionCoprocessor::decodeToTurretAimData(
    const SerialMessage& message,
    TurretAimData* aimData)
{
    if (message.length != AIM_DATA_MESSAGE_SIZE)
    {
        return false;
    }
    
    convertFromLittleEndian(&aimData->xPos, &message.data[AIM_DATA_MESSAGE_X_POSITION_OFFSET]);
    convertFromLittleEndian(&aimData->yPos, &message.data[AIM_DATA_MESSAGE_Y_POSITION_OFFSET]);
    convertFromLittleEndian(&aimData->zPos, &message.data[AIM_DATA_MESSAGE_Z_POSITION_OFFSET]);
    convertFromLittleEndian(&aimData->xVel, &message.data[AIM_DATA_MESSAGE_X_VELOCITY_OFFSET]);
    convertFromLittleEndian(&aimData->yVel, &message.data[AIM_DATA_MESSAGE_Y_VELOCITY_OFFSET]);
    convertFromLittleEndian(&aimData->zVel, &message.data[AIM_DATA_MESSAGE_Z_VELOCITY_OFFSET]);
    convertFromLittleEndian(&aimData->xAcc, &message.data[AIM_DATA_MESSAGE_X_ACCELERATION_OFFSET]);
    convertFromLittleEndian(&aimData->yAcc, &message.data[AIM_DATA_MESSAGE_Y_ACCELERATION_OFFSET]);
    convertFromLittleEndian(&aimData->zAcc, &message.data[AIM_DATA_MESSAGE_Z_ACCELERATION_OFFSET]);
    convertFromLittleEndian(&aimData->timestamp, &message.data[AIM_DATA_MESSAGE_TIMESTAMP_MICROS_OFFSET]);
    convertFromLittleEndian(&aimData->hasTarget, &message.data[AIM_DATA_MESSAGE_HAS_TARGET_OFFSET]);

    return true;
}

void VisionCoprocessor::sendMessage()
{
    sendOdometryData();
}

void VisionCoprocessor::sendOdometryData()
{
    // TODO: add odometry data
    convertToLittleEndian(
        static_cast<int32_t>(turretMCBCanComm->getYaw()),
        txMessage.data + ODOMETRY_DATA_MESSAGE_TURRET_YAW_OFFSET);
    convertToLittleEndian(
        static_cast<int32_t>(turretMCBCanComm->getPitch()),
        txMessage.data + ODOMETRY_DATA_MESSAGE_TURRET_PITCH_OFFSET);
    txMessage.type = CV_MESSAGE_ODOMETRY_DATA;
    txMessage.length = ODOMETRY_DATA_MESSAGE_SIZE;
    send();
}

}  // namespace serial
}  // namespace aruwsrc
