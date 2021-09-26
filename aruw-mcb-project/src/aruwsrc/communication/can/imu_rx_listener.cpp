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

#include "imu_rx_listener.hpp"

#include "tap/architecture/endianness_wrappers.hpp"

#include "modm/architecture/interface/can.hpp"

namespace aruwsrc::can
{
ImuRxListener::ImuRxListener(tap::Drivers* drivers)
    : drivers(drivers),
      angleGyroMessageHandler(
          drivers,
          ANGLE_GYRO_MESSAGE_CAN_ID,
          IMU_MSG_CAN_BUS,
          this,
          &ImuRxListener::handleAngleGyroMessage)
{
}

ImuRxListener::ImuRxHandler::ImuRxHandler(
    tap::Drivers* drivers,
    uint32_t id,
    tap::can::CanBus cB,
    ImuRxListener* msgHandler,
    ImuRxListenerFunc funcToCall)
    : CanRxListener(drivers, id, cB),
      msgHandler(msgHandler),
      funcToCall(funcToCall)
{
}

void ImuRxListener::init() { angleGyroMessageHandler.attachSelfToRxHandler(); }

void ImuRxListener::ImuRxHandler::processMessage(const modm::can::Message& message)
{
    (msgHandler->*funcToCall)(message);
}

void ImuRxListener::handleAngleGyroMessage(const modm::can::Message& message)
{
    uint16_t rawYaw;
    int16_t rawPitch;
    tap::arch::convertFromLittleEndian(&rawYaw, message.data);
    tap::arch::convertFromLittleEndian(&rawGz, message.data + 2);
    tap::arch::convertFromLittleEndian(&rawPitch, message.data + 4);
    tap::arch::convertFromLittleEndian(&rawGx, message.data + 6);

    yaw = static_cast<float>(rawYaw) * ANGLE_DATA_FIXED_POINT_PRECISION;
    pitch = static_cast<float>(rawPitch) * ANGLE_DATA_FIXED_POINT_PRECISION;

    imuConnectedTimeout.restart(DISCONNECT_TIMEOUT_PERIOD);
}
}  // namespace aruwsrc::can
