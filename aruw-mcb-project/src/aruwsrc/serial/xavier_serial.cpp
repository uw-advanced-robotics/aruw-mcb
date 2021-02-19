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

#include <cstring>

#include <aruwlib/Drivers.hpp>
#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "aruwsrc/control/chassis/chassis_subsystem.hpp"

namespace aruwsrc
{
namespace serial
{
XavierSerial::XavierSerial(aruwlib::Drivers* drivers)
    : aruwlib::serial::DJISerial<>(drivers, aruwlib::serial::Uart::UartPort::Uart2),
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
            if (decodeToTurrentAimData(completeMessage, &lastAimData))
            {
                aimDataValid = true;
            }
            return;
        }
        case CV_MESSAGE_TYPE_TRACKING_REQUEST_ACKN:
        {
            if (AutoAimRequest.currAimState == AUTO_AIM_REQUEST_SENT)
            {
                AutoAimRequest.currAimState = AUTO_AIM_REQUEST_ACKNOWLEDGED;
            }
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

    // TODO redo this with proper endianness converter
    int32_t raw_pitch =
        *(reinterpret_cast<const int32_t*>(message.data + AIM_DATA_MESSAGE_PITCH_OFFSET));
    int32_t raw_yaw =
        *(reinterpret_cast<const int32_t*>(message.data + AIM_DATA_MESSAGE_YAW_OFFSET));

    aimData->pitch = static_cast<float>(raw_pitch) / 100.0f;
    aimData->yaw = static_cast<float>(raw_yaw) / 100.0f;
    aimData->hasTarget = message.data[AIM_DATA_MESSAGE_HAS_TARGET];
    aimData->timestamp = message.messageTimestamp;

    return true;
}

void XavierSerial::sendMessage()
{
    isCvOnline = !cvOfflineTimeout.isExpired();

    if (!isCvOnline)
    {
        aimDataValid = false;
    }

    // Send robot measurements every time
    // TODO incrementing a counter to alternate between messages seems weird since the robot id and auto aim requests aren't sent often, so I removed it altogether, is this bad? We should test for real
    sendRobotMeasurements();
    sendRobotID();
    sendAutoAimRequest();
}

bool XavierSerial::sendRobotMeasurements()
{
    if (chassisSub == nullptr || turretSub == nullptr)
    {
        return false;
    }

    txMessage.data[0] = static_cast<uint8_t>(chassisSub->getLeftFrontRpmActual() >> 8);
    txMessage.data[1] = static_cast<uint8_t>(chassisSub->getLeftFrontRpmActual());
    // etc, use helper endian converter func

    int32_t fixedPointPitch = static_cast<int32_t>(turretSub->getPitchAngle().getValue() * 1000.0f);
    int32_t fixedPointYaw = static_cast<int32_t>(turretSub->getYawAngle().getValue() * 1000.0f);
    // etc, user helper func, convertToLittleEndian(txMessage.data + sizeof(int16_t) * 4, static_cast<int32_t>(turretSub->getYawAngle().getValue() * 1000)
    // replace above

    // Do the same for imu data

    txMessage.type = static_cast<uint8_t>(CV_MESSAGE_TYPE_ROBOT_DATA);
    txMessage.length = 4 * sizeof(int16_t) + (2 + 9) * sizeof(int32_t);
    return send();
}

void XavierSerial::beginAutoAim()
{
    AutoAimRequest.requestType = true;
    AutoAimRequest.currAimState = AUTO_AIM_REQUEST_QUEUED;
}

void XavierSerial::stopAutoAim()
{
    AutoAimRequest.requestType = true;
    AutoAimRequest.currAimState = AUTO_AIM_REQUEST_QUEUED;
}

bool XavierSerial::sendAutoAimRequest()
{
    if (AutoAimRequest.currAimState == AUTO_AIM_REQUEST_ACKNOWLEDGED)
    {
        AutoAimRequest.sendAimRequestTimeout.stop();
        AutoAimRequest.currAimState = AUTO_AIM_REQUEST_COMPLETE;
    }
    else if (
        AutoAimRequest.currAimState == AUTO_AIM_REQUEST_QUEUED ||
        (AutoAimRequest.currAimState == AUTO_AIM_REQUEST_SENT &&
         AutoAimRequest.sendAimRequestTimeout.execute()))
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
