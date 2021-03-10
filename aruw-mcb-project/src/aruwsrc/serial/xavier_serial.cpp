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
#include <aruwlib/architecture/endianness_wrappers.hpp>

#include "aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"

using namespace aruwlib::arch;

#define SEND_MSG(state, func, currTxMsg)                                                     \
    case state:                                                                              \
    {                                                                                        \
        auto res = func();                                                                   \
        if (res == SUCCESS)                                                                  \
        {                                                                                    \
            currTxMsg = static_cast<TxMessageTypes>((currTxMsg + 1) % CV_MESSAGE_TYPE_SIZE); \
            break;                                                                           \
        }                                                                                    \
        else if (res == DID_NOT_SEND)                                                        \
        {                                                                                    \
            currTxMsg = static_cast<TxMessageTypes>((currTxMsg + 1) % CV_MESSAGE_TYPE_SIZE); \
            modm_fallthrough;                                                                \
        }                                                                                    \
        else                                                                                 \
        {                                                                                    \
            break;                                                                           \
        }                                                                                    \
    }

#define BEGIN_SEND_MSG(state, func, currTxMsg)                \
    {                                                         \
        TxMessageTypes prevTxMessageType = currTxMessageType; \
        auto initialFunc = &XavierSerial::func;               \
        auto initialState = state;                            \
        switch (currTxMsg)                                    \
        {                                                     \
            SEND_MSG(state, func, currTxMsg)

#define END_SEND_MSG()                        \
    case CV_MESSAGE_TYPE_SIZE:                \
        if (prevTxMessageType > initialState) \
        {                                     \
            (this->*initialFunc)();           \
        }                                     \
        break;                                \
        }                                     \
        }

namespace aruwsrc
{
namespace serial
{
XavierSerial::XavierSerial(
    aruwlib::Drivers* drivers,
    const turret::TurretSubsystem* turretSub,
    const chassis::ChassisSubsystem* chassisSub)
    : aruwlib::serial::DJISerial(drivers, aruwlib::serial::Uart::UartPort::Uart2),
      lastAimData(),
      aimDataValid(false),
      isCvOnline(false),
      turretSub(turretSub),
      chassisSub(chassisSub)
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
                AutoAimRequest.currAimState = AUTO_AIM_REQUEST_COMPLETE;
                AutoAimRequest.sendAimRequestTimeout.stop();
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

    uint16_t rawPitch, rawYaw;
    convertFromLittleEndian(&rawPitch, message.data + AIM_DATA_MESSAGE_PITCH_OFFSET);
    convertFromLittleEndian(&rawYaw, message.data + AIM_DATA_MESSAGE_YAW_OFFSET);

    aimData->pitch = static_cast<float>(rawPitch) * FIXED_POINT_PRECISION;
    aimData->yaw = static_cast<float>(rawYaw) * FIXED_POINT_PRECISION;
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

    BEGIN_SEND_MSG(CV_MESSAGE_TYPE_ROBOT_DATA, sendRobotMeasurements, currTxMessageType);
    SEND_MSG(CV_MESSAGE_TYPE_ROBOT_ID, sendRobotID, currTxMessageType);
    SEND_MSG(CV_MESSAGE_TYPE_AUTO_AIM_REQUEST, sendAutoAimRequest, currTxMessageType);
    END_SEND_MSG();
}

XavierSerial::TxMessageState XavierSerial::sendRobotMeasurements()
{
    if (chassisSub == nullptr || turretSub == nullptr)
    {
        return DID_NOT_SEND;
    }

    // Chassis data
    convertToLittleEndian(
        chassisSub->getRightFrontRpmActual(),
        txMessage.data);
    convertToLittleEndian(chassisSub->getLeftFrontRpmActual(), txMessage.data + sizeof(int16_t));
    convertToLittleEndian(chassisSub->getLeftBackRpmActual(), txMessage.data + 2 * sizeof(int16_t));
    convertToLittleEndian(
        chassisSub->getRightBackRpmActual(),
        txMessage.data + 3 * sizeof(int16_t));

    // Turret data
    convertToLittleEndian(
        static_cast<uint16_t>(turretSub->getPitchAngle().getValue() / FIXED_POINT_PRECISION),
        txMessage.data + TURRET_DATA_OFFSET);
    convertToLittleEndian(
        static_cast<uint16_t>(turretSub->getYawAngle().getValue() / FIXED_POINT_PRECISION),
        txMessage.data + TURRET_DATA_OFFSET + sizeof(uint16_t));

    // IMU data
    convertToLittleEndian(
        static_cast<int32_t>(drivers->mpu6500.getGx() / FIXED_POINT_PRECISION),
        txMessage.data + IMU_DATA_OFFSET);
    convertToLittleEndian(
        static_cast<int32_t>(drivers->mpu6500.getGy() / FIXED_POINT_PRECISION),
        txMessage.data + IMU_DATA_OFFSET + sizeof(int32_t));
    convertToLittleEndian(
        static_cast<int32_t>(drivers->mpu6500.getGz() / FIXED_POINT_PRECISION),
        txMessage.data + IMU_DATA_OFFSET + 2 * sizeof(int32_t));
    convertToLittleEndian(
        static_cast<int16_t>(drivers->mpu6500.getAx() / FIXED_POINT_PRECISION),
        txMessage.data + IMU_DATA_OFFSET + 3 * sizeof(int32_t));
    convertToLittleEndian(
        static_cast<int16_t>(drivers->mpu6500.getAy() / FIXED_POINT_PRECISION),
        txMessage.data + IMU_DATA_OFFSET + 3 * sizeof(int32_t) + sizeof(int16_t));
    convertToLittleEndian(
        static_cast<int16_t>(drivers->mpu6500.getAz() / FIXED_POINT_PRECISION),
        txMessage.data + IMU_DATA_OFFSET + 3 * sizeof(int32_t) + 2 * sizeof(int16_t));
    convertToLittleEndian(
        static_cast<uint16_t>(drivers->mpu6500.getYaw() / FIXED_POINT_PRECISION),
        txMessage.data + IMU_DATA_OFFSET + 3 * sizeof(int32_t) + 3 * sizeof(int16_t));
    convertToLittleEndian(
        static_cast<int16_t>(drivers->mpu6500.getPitch() / FIXED_POINT_PRECISION),
        txMessage.data + IMU_DATA_OFFSET + 3 * sizeof(int32_t) + 4 * sizeof(int16_t));
    convertToLittleEndian(
        static_cast<int16_t>(drivers->mpu6500.getRoll() / FIXED_POINT_PRECISION),
        txMessage.data + IMU_DATA_OFFSET + 3 * sizeof(int32_t) + 5 * sizeof(int16_t));

    txMessage.type = static_cast<uint8_t>(CV_MESSAGE_TYPE_ROBOT_DATA);
    txMessage.length = ROBOT_DATA_MSG_SIZE;
    return send() ? SUCCESS : FAIL;
}

void XavierSerial::beginAutoAim()
{
    AutoAimRequest.requestType = true;
    AutoAimRequest.currAimState = AUTO_AIM_REQUEST_QUEUED;
}

void XavierSerial::stopAutoAim()
{
    AutoAimRequest.requestType = false;
    AutoAimRequest.currAimState = AUTO_AIM_REQUEST_QUEUED;
}

XavierSerial::TxMessageState XavierSerial::sendAutoAimRequest()
{
    if (AutoAimRequest.currAimState == AUTO_AIM_REQUEST_QUEUED ||
        (AutoAimRequest.currAimState == AUTO_AIM_REQUEST_SENT &&
         AutoAimRequest.sendAimRequestTimeout.isExpired()))
    {
        txMessage.data[0] = AutoAimRequest.requestType;
        txMessage.length = 1;
        txMessage.type = CV_MESSAGE_TYPE_AUTO_AIM_REQUEST;
        if (send())
        {
            AutoAimRequest.currAimState = AUTO_AIM_REQUEST_SENT;
            AutoAimRequest.sendAimRequestTimeout.restart(AUTO_AIM_REQUEST_SEND_PERIOD_MS);
            return SUCCESS;
        }
        return FAIL;
    }
    return DID_NOT_SEND;
}

XavierSerial::TxMessageState XavierSerial::sendRobotID()
{
    if (txRobotIdTimeout.execute() || !robotIdSendSucceeded)
    {
        txMessage.data[0] = static_cast<uint8_t>(drivers->refSerial.getRobotData().robotId);
        txMessage.length = 1;
        txMessage.type = CV_MESSAGE_TYPE_ROBOT_ID;
        robotIdSendSucceeded = send();
        return robotIdSendSucceeded ? SUCCESS : FAIL;
    }
    return DID_NOT_SEND;
}
}  // namespace serial
}  // namespace aruwsrc
