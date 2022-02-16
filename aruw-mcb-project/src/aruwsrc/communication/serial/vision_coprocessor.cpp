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
#include "vision_coprocessor.hpp"

#include "aruwsrc/drivers.hpp"

using namespace tap::arch;
using namespace tap::communication::serial;

namespace aruwsrc
{
namespace serial
{
VisionCoprocessor::VisionCoprocessor(aruwsrc::Drivers* drivers)
    : DJISerial(drivers, VISION_COPROCESSOR_RX_PORT),
      lastAimData(),
      turretMCBCanComm(&drivers->turretMCBCanComm),
      odometryInterface(nullptr)
{
}

void VisionCoprocessor::initializeCV()
{
    cvOfflineTimeout.restart(TIME_OFFLINE_CV_AIM_DATA_MS);
    drivers->uart.init<VISION_COPROCESSOR_TX_UART_PORT, 1'000'000>();
    initialize();
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

void VisionCoprocessor::sendMessage() { sendOdometryData(); }

bool VisionCoprocessor::isCvOnline() { return !cvOfflineTimeout.isExpired(); }

void VisionCoprocessor::sendOdometryData()
{
    DJISerial::SerialMessage<sizeof(OdometryData)> odometryMessage;

    modm::Location2D<float> location = modm::Location2D<float>();

    if (odometryInterface != nullptr)
    {
        location = odometryInterface->getCurrentLocation2D();
    }

    odometryMessage.header.headByte = 0xa5;
    odometryMessage.header.dataLength = sizeof(OdometryData);
    odometryMessage.header.seq = 0;
    odometryMessage.header.CRC8 = tap::algorithms::calculateCRC8(
        reinterpret_cast<uint8_t*>(&odometryMessage),
        sizeof(odometryMessage.header) - 1);

    odometryMessage.messageType = CV_MESSAGE_TYPE_ODOMETRY_DATA;

    OdometryData* odometryData = reinterpret_cast<OdometryData*>(&odometryMessage.data);

    odometryData->chassisX = location.getX();
    odometryData->chassisY = location.getY();
    odometryData->chassisZ = 0.0f;
    odometryData->turretPitch = turretMCBCanComm->getPitch();
    odometryData->turretYaw = turretMCBCanComm->getYaw();
    odometryData->timestamp = tap::arch::clock::getTimeMicroseconds();

    odometryMessage.CRC16 = tap::algorithms::calculateCRC16(
        reinterpret_cast<uint8_t*>(&odometryMessage),
        sizeof(odometryMessage) - 2);

    drivers->uart.write(
        VISION_COPROCESSOR_TX_UART_PORT,
        reinterpret_cast<uint8_t*>(&odometryMessage),
        sizeof(odometryMessage));
}

}  // namespace serial
}  // namespace aruwsrc
