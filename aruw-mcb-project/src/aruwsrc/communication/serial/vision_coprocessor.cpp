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

#include "vision_coprocessor.hpp"

#include "tap/errors/create_errors.hpp"

#include "aruwsrc/drivers.hpp"

using namespace tap::arch;
using namespace tap::communication::serial;
using tap::arch::clock::getTimeMicroseconds;

namespace aruwsrc
{
namespace serial
{
VisionCoprocessor::VisionCoprocessor(aruwsrc::Drivers* drivers)
    : DJISerial(drivers, VISION_COPROCESSOR_RX_UART_PORT),
      lastAimData(),
      odometryInterface(nullptr),
      turretOrientationInterface(nullptr)
{
}

void VisionCoprocessor::initializeCV()
{
    cvOfflineTimeout.restart(TIME_OFFLINE_CV_AIM_DATA_MS);
#if defined(TARGET_HERO)
    drivers->uart.init<VISION_COPROCESSOR_TX_UART_PORT, 900'000>();
    drivers->uart.init<VISION_COPROCESSOR_RX_UART_PORT, 900'000>();
#else
    drivers->uart.init<VISION_COPROCESSOR_TX_UART_PORT, 1'000'000>();
    drivers->uart.init<VISION_COPROCESSOR_RX_UART_PORT, 1'000'000>();
#endif
}

void VisionCoprocessor::messageReceiveCallback(const ReceivedSerialMessage& completeMessage)
{
    cvOfflineTimeout.restart(TIME_OFFLINE_CV_AIM_DATA_MS);
    switch (completeMessage.messageType)
    {
        case CV_MESSAGE_TYPE_TURRET_AIM:
        {
            decodeToTurretAimData(completeMessage, &lastAimData);
            return;
        }
        case CV_MESSAGE_TYPE_TIME_SYNC_REQ:
        {
            decodeAndSendTimeSyncMessage(completeMessage);
            return;
        }
        default:
            return;
    }
}

bool VisionCoprocessor::decodeToTurretAimData(
    const ReceivedSerialMessage& message,
    TurretAimData* aimData)
{
    if (message.header.dataLength != sizeof(*aimData))
    {
        return false;
    }
    memcpy(aimData, &message.data, sizeof(*aimData));
    return true;
}

void VisionCoprocessor::decodeAndSendTimeSyncMessage(const ReceivedSerialMessage&)
{
    DJISerial::SerialMessage<sizeof(uint32_t)> timeSyncResponseMessage;
    timeSyncResponseMessage.messageType = CV_MESSAGE_TYPE_TIME_SYNC_RESP;
    *reinterpret_cast<uint32_t*>(timeSyncResponseMessage.data) = getTimeMicroseconds();
    timeSyncResponseMessage.setCRC16();
    drivers->uart.write(
        VISION_COPROCESSOR_TX_UART_PORT,
        reinterpret_cast<uint8_t*>(&timeSyncResponseMessage),
        sizeof(timeSyncResponseMessage));
}

void VisionCoprocessor::sendMessage()
{
    sendOdometryData();
    sendRobotTypeData();
}

bool VisionCoprocessor::isCvOnline() { return !cvOfflineTimeout.isExpired(); }

void VisionCoprocessor::sendShutdownMessage()
{
    DJISerial::SerialMessage<1> shutdownMessage;

    shutdownMessage.messageType = CV_MESSAGE_TYPE_SHUTDOWN;
    shutdownMessage.data[0] = 0;
    shutdownMessage.setCRC16();
    drivers->uart.write(
        VISION_COPROCESSOR_TX_UART_PORT,
        reinterpret_cast<uint8_t*>(&shutdownMessage),
        sizeof(shutdownMessage));
}

void VisionCoprocessor::sendRebootMessage()
{
    DJISerial::SerialMessage<1> rebootMessage;
    rebootMessage.messageType = CV_MESSAGE_TYPE_REBOOT;
    rebootMessage.data[0] = 0;
    rebootMessage.setCRC16();
    drivers->uart.write(
        VISION_COPROCESSOR_TX_UART_PORT,
        reinterpret_cast<uint8_t*>(&rebootMessage),
        sizeof(rebootMessage));
}

void VisionCoprocessor::sendOdometryData()
{
    DJISerial::SerialMessage<sizeof(OdometryData)> odometryMessage;

    modm::Location2D<float> location = modm::Location2D<float>();

    if (odometryInterface != nullptr)
    {
        location = odometryInterface->getCurrentLocation2D();
    }

    odometryMessage.messageType = CV_MESSAGE_TYPE_ODOMETRY_DATA;

    OdometryData* odometryData = reinterpret_cast<OdometryData*>(&odometryMessage.data);

    odometryData->chassisX = location.getX();
    odometryData->chassisY = location.getY();
    odometryData->chassisZ = 0.0f;
    if (turretOrientationInterface != nullptr)
    {
        odometryData->turretPitch = turretOrientationInterface->getWorldPitch();
        odometryData->turretYaw = turretOrientationInterface->getWorldYaw();
        odometryData->turretTimestamp = turretOrientationInterface->getLastMeasurementTimeMicros();
    }
    else
    {
        odometryData->turretPitch = 0.0f;
        odometryData->turretYaw = 0.0f;
        odometryData->turretTimestamp = 0;
        RAISE_ERROR(drivers, "turret interface not attached");
    }

    odometryMessage.setCRC16();

    drivers->uart.write(
        VISION_COPROCESSOR_TX_UART_PORT,
        reinterpret_cast<uint8_t*>(&odometryMessage),
        sizeof(odometryMessage));
}

void VisionCoprocessor::sendRobotTypeData()
{
    if (sendRobotIdTimeout.execute())
    {
        DJISerial::SerialMessage<1> robotTypeMessage;
        robotTypeMessage.messageType = CV_MESSAGE_TYPE_ROBOT_ID;
        robotTypeMessage.data[0] = static_cast<uint8_t>(drivers->refSerial.getRobotData().robotId);
        robotTypeMessage.setCRC16();
        drivers->uart.write(
            VISION_COPROCESSOR_TX_UART_PORT,
            reinterpret_cast<uint8_t*>(&robotTypeMessage),
            sizeof(robotTypeMessage));
    }
}

void VisionCoprocessor::sendSelectNewTargetMessage()
{
    DJISerial::SerialMessage<4> selectNewTargetMessage;
    selectNewTargetMessage.messageType = CV_MESSAGE_TYPE_SELECT_NEW_TARGET;
    selectNewTargetMessage.data[0] = 0;
    selectNewTargetMessage.data[1] = 0;
    selectNewTargetMessage.data[2] = 0;
    selectNewTargetMessage.data[3] = 0;
    selectNewTargetMessage.setCRC16();
    drivers->uart.write(
        VISION_COPROCESSOR_TX_UART_PORT,
        reinterpret_cast<uint8_t*>(&selectNewTargetMessage),
        sizeof(selectNewTargetMessage));
}

}  // namespace serial
}  // namespace aruwsrc
