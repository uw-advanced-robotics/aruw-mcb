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

#include "xavier_serial.hpp"
#include <aruwlib/Drivers.hpp>

#include <cstring>

namespace aruwlib
{
namespace serial
{
XavierSerial::XavierSerial(Drivers* drivers)
    : DJISerial(drivers, Uart::UartPort::Uart2),
      lastAimData(),
      aimDataValid(false),
      isCvOnline(false)
{
}

void XavierSerial::initializeCV()
{
    txRobotIdTimeout.restart(TIME_BETWEEN_ROBOT_ID_SEND_MS);
    cvOfflineTimeout.restart(TIME_OFFLINE_CV_AIM_DATA_MS);
    initialize();
}

void XavierSerial::messageReceiveCallback(const SerialMessage& completeMessage)
{
    cvOfflineTimeout.restart(TIME_OFFLINE_CV_AIM_DATA_MS);
    switch (completeMessage.type)
    {
        case CV_MESSAGE_TYPE_TURRET_AIM:
        {
            TurretAimData aimData;
            if (decodeToTurrentAimData(completeMessage, &aimData))
            {
                lastAimData = aimData;
                aimDataValid = true;
            }
            return;
        }
        default:
            return;
    }
}

bool XavierSerial::decodeToTurrentAimData(const SerialMessage& message, TurretAimData* aimData)
{
    if (message.length != AIM_DATA_MESSAGE_SIZE)
    {
        return false;
    }

    int16_t raw_pitch =
        *(reinterpret_cast<const int16_t*>(message.data + AIM_DATA_MESSAGE_PITCH_OFFSET));
    int16_t raw_yaw =
        *(reinterpret_cast<const int16_t*>(message.data + AIM_DATA_MESSAGE_YAW_OFFSET));

    bool raw_has_target = message.data[AIM_DATA_MESSAGE_HAS_TARGET];

    aimData->pitch = static_cast<float>(raw_pitch) / 100.0f;
    aimData->yaw = static_cast<float>(raw_yaw) / 100.0f;
    aimData->hasTarget = raw_has_target;
    aimData->timestamp = message.messageTimestamp;

    return true;
}

void XavierSerial::sendMessage(const RobotData& robotData)
{
    isCvOnline = !cvOfflineTimeout.isExpired();

    if (!isCvOnline)
    {
        aimDataValid = false;
    }

    // Send robot measurements every time
    sendRobotMeasurements(robotData);
    sendRobotID();
    sendAutoAimRequest();
}

void XavierSerial::beginTargetTracking()
{
    AutoAimRequest.requestType = true;
    AutoAimRequest.currAimState = AUTO_AIM_REQUEST_QUEUED;
}

void XavierSerial::stopTargetTracking()
{
    AutoAimRequest.requestType = true;
    AutoAimRequest.currAimState = AUTO_AIM_REQUEST_QUEUED;
}

bool XavierSerial::sendRobotMeasurements(const RobotData& robotData)
{
    static constexpr int TURRET_DATA_OFFSET = sizeof(robotData.IMU);
    static constexpr int CHASSIS_DATA_OFFSET = sizeof(robotData.IMU) + sizeof(robotData.Turret);
}

bool XavierSerial::sendAutoAimRequest()
{
    if (AutoAimRequest.currAimState == AUTO_AIM_REQUEST_RECEIVED)
    {
        AutoAimRequest.sendAimRequestTimeout.stop();
        AutoAimRequest.currAimState = AUTO_AIM_REQUEST_COMPLETE;
    }
    else if (AutoAimRequest.currAimState == AUTO_AIM_REQUEST_QUEUED ||
             (AutoAimRequest.currAimState == AUTO_AIM_REQUEST_SENT && AutoAimRequest.sendAimRequestTimeout.execute()))
    {
        txMessage.data[0] = AutoAimRequest.requestType;
        txMessage.length = 1;
        txMessage.type = CV_MESSAGE_TYPE_AUTO_AIM_REQUEST;
        if (send())
        {
            AutoAimRequest.currAimState = AUTO_AIM_REQUEST_SENT;
            AutoAimRequest.sendAimRequestTimeout.restart(AUTO_AIM_REQUEST_SEND_PERIOD_MS);
            return true;
        }
        return false;
    }
    return true;
}

bool XavierSerial::sendRobotID()
{
    if (txRobotIdTimeout.execute())
    {
        txMessage.data[0] = static_cast<uint8_t>(drivers->refSerial.getRobotData().robotId);
        txMessage.length = 1;
        txMessage.type = CV_MESSAGE_TYPE_ROBOT_ID;
        return send();
    }
    return true;
}
}  // namespace serial
}  // namespace aruwlib
