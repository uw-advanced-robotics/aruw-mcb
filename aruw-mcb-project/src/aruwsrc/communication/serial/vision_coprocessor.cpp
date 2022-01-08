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
      isCvOnline(false),
      turretMCBCanComm(&drivers->turretMCBCanComm)
{
}

void VisionCoprocessor::initializeCV()
{
    txRobotIdTimeout.restart(TIME_BETWEEN_ROBOT_ID_SEND_MS);
    cvOfflineTimeout.restart(TIME_OFFLINE_CV_AIM_DATA_MS);
    initialize();
}

void VisionCoprocessor::messageReceiveCallback(const SerialMessage& completeMessage)
{
    cvOfflineTimeout.restart(TIME_OFFLINE_CV_AIM_DATA_MS);
    switch (completeMessage.type)
    {
        case CV_MESSAGE_TYPE_TURRET_AIM:
        {

            if (decodeToTurretAimData(completeMessage, &lastAimData))
            {
                aimDataValid = true;

                if (AutoAimRequest.currAimState == AUTO_AIM_REQUEST_SENT)
                {
                    AutoAimRequest.currAimState = AUTO_AIM_REQUEST_COMPLETE;
                    AutoAimRequest.sendAimRequestTimeout.stop();
                }
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

    uint32_t xPos, yPos, zPos, xVel, yVel, zVel, xAcc, yAcc, zAcc, timestamp;
    uint8_t hasTarget;
    convertFromLittleEndian(&xPos, message.data + AIM_DATA_MESSAGE_X_POSITION_OFFSET);
    convertFromLittleEndian(&yPos, message.data + AIM_DATA_MESSAGE_Y_POSITION_OFFSET);
    convertFromLittleEndian(&zPos, message.data + AIM_DATA_MESSAGE_Z_POSITION_OFFSET);
    convertFromLittleEndian(&xVel, message.data + AIM_DATA_MESSAGE_X_VELOCITY_OFFSET);
    convertFromLittleEndian(&yVel, message.data + AIM_DATA_MESSAGE_Y_VELOCITY_OFFSET);
    convertFromLittleEndian(&zVel, message.data + AIM_DATA_MESSAGE_Z_VELOCITY_OFFSET);
    convertFromLittleEndian(&xAcc, message.data + AIM_DATA_MESSAGE_X_ACCELERATION_OFFSET);
    convertFromLittleEndian(&yAcc, message.data + AIM_DATA_MESSAGE_Y_ACCELERATION_OFFSET);
    convertFromLittleEndian(&zAcc, message.data + AIM_DATA_MESSAGE_Z_ACCELERATION_OFFSET);
    convertFromLittleEndian(&timestamp, message.data + AIM_DATA_MESSAGE_TIMESTAMP_MICROS_OFFSET);

    aimData->xPos = *reinterpret_cast<float *>(xPos);
    aimData->yPos = *reinterpret_cast<float *>(yPos);
    aimData->zPos = *reinterpret_cast<float *>(zPos);
    aimData->xVel = *reinterpret_cast<float *>(xVel);
    aimData->yVel = *reinterpret_cast<float *>(yVel);
    aimData->zVel = *reinterpret_cast<float *>(zVel);
    aimData->xAcc = *reinterpret_cast<float *>(xAcc);
    aimData->yAcc = *reinterpret_cast<float *>(yAcc);
    aimData->zAcc = *reinterpret_cast<float *>(zAcc);
    aimData->timestamp = *reinterpret_cast<float *>(timestamp);
    aimData->hasTarget = message.data[AIM_DATA_MESSAGE_HAS_TARGET_OFFSET];

    return true;
}

bool VisionCoprocessor::sendMessage()
{
    while (true)
    {
        isCvOnline = !cvOfflineTimeout.isExpired();
        aimDataValid &= isCvOnline;
        sendOdometryData();
    }
}

bool VisionCoprocessor::sendOdometryData()
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

void VisionCoprocessor::beginAutoAim()
{
    AutoAimRequest.autoAimRequest = true;
    AutoAimRequest.currAimState = AUTO_AIM_REQUEST_QUEUED;
}

void VisionCoprocessor::stopAutoAim()
{
    AutoAimRequest.autoAimRequest = false;
    AutoAimRequest.currAimState = AUTO_AIM_REQUEST_QUEUED;
}

bool VisionCoprocessor::sendAutoAimRequest()
{
    // If there is an auto aim request queued or if the request aim
    // timeout is expired (we haven't received a auto aim message),
    // send an auto aim request message.
    if (AutoAimRequest.currAimState == AUTO_AIM_REQUEST_QUEUED ||
        (AutoAimRequest.currAimState == AUTO_AIM_REQUEST_SENT &&
         AutoAimRequest.sendAimRequestTimeout.isExpired()))
    {
        txMessage.data[0] = AutoAimRequest.autoAimRequest;
        txMessage.length = 1;
        txMessage.type = CV_MESSAGE_TYPE_AUTO_AIM_REQUEST;

        if (send())
        {
            if (AutoAimRequest.autoAimRequest)
            {
                AutoAimRequest.currAimState = AUTO_AIM_REQUEST_SENT;
                AutoAimRequest.sendAimRequestTimeout.restart(AUTO_AIM_REQUEST_SEND_PERIOD_MS);
            }
            else
            {
                AutoAimRequest.currAimState = AUTO_AIM_REQUEST_COMPLETE;
            }
        }
    }
}

}  // namespace serial
}  // namespace aruwsrc
