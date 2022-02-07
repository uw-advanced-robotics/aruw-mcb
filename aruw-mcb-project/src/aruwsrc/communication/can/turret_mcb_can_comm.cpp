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

#include "turret_mcb_can_comm.hpp"

#include "tap/architecture/endianness_wrappers.hpp"

#include "aruwsrc/drivers.hpp"
#include "modm/architecture/interface/can.hpp"

namespace aruwsrc::can
{
TurretMCBCanComm::TurretMCBCanComm(aruwsrc::Drivers* drivers)
    : drivers(drivers),
      angleGyroMessageHandler(
          drivers,
          ANGLE_GYRO_RX_CAN_ID,
          IMU_MSG_CAN_BUS,
          this,
          &TurretMCBCanComm::handleAngleGyroMessage),
      turretStatusRxHandler(
          drivers,
          TURRET_STATUS_RX_CAN_ID,
          IMU_MSG_CAN_BUS,
          this,
          &TurretMCBCanComm::handleTurretMessage),
      openHopperCover(false),
      calibrateImu(false),
      sendMcbDataTimer(SEND_MCB_DATA_TIMEOUT)
{
}

void TurretMCBCanComm::init()
{
    angleGyroMessageHandler.attachSelfToRxHandler();
    turretStatusRxHandler.attachSelfToRxHandler();
}

void TurretMCBCanComm::sendData()
{
    if (sendMcbDataTimer.execute())
    {
        modm::can::Message txMsg(TURRET_MCB_TX_CAN_ID, 1);
        txMsg.setExtended(false);
        txMsg.data[0] = (static_cast<uint8_t>(openHopperCover) & 0b1) |
                        ((static_cast<uint8_t>(calibrateImu) & 0b1) << 1);
        drivers->can.sendMessage(tap::can::CanBus::CAN_BUS1, txMsg);

        // set this calibrate flag to false so the calibrate command is only sent once
        calibrateImu = false;
    }
}

void TurretMCBCanComm::handleAngleGyroMessage(const modm::can::Message& message)
{
    // Update light to indicate IMU message received and turret controller running.
    imuMessageReceivedLEDBlinkCounter = (imuMessageReceivedLEDBlinkCounter + 1) % 100;
    drivers->leds.set(tap::gpio::Leds::Green, imuMessageReceivedLEDBlinkCounter > 50);

    int16_t rawYaw;
    int16_t rawPitch;
    tap::arch::convertFromLittleEndian(&rawYaw, message.data);
    tap::arch::convertFromLittleEndian(&rawYawVelocity, message.data + 2);
    tap::arch::convertFromLittleEndian(&rawPitch, message.data + 4);
    tap::arch::convertFromLittleEndian(&rawPitchVelocity, message.data + 6);

    yaw = static_cast<float>(rawYaw) * ANGLE_FIXED_POINT_PRECISION;
    pitch = static_cast<float>(rawPitch) * ANGLE_FIXED_POINT_PRECISION;

    imuConnectedTimeout.restart(DISCONNECT_TIMEOUT_PERIOD);
}

void TurretMCBCanComm::handleTurretMessage(const modm::can::Message& message)
{
    limitSwitchDepressed = message.data[0] & 0b1;
}

TurretMCBCanComm::ImuRxHandler::ImuRxHandler(
    aruwsrc::Drivers* drivers,
    uint32_t id,
    tap::can::CanBus cB,
    TurretMCBCanComm* msgHandler,
    CanCommListenerFunc funcToCall)
    : CanRxListener(drivers, id, cB),
      msgHandler(msgHandler),
      funcToCall(funcToCall)
{
}

TurretMCBCanComm::TurretStatusRxHandler::TurretStatusRxHandler(
    aruwsrc::Drivers* drivers,
    uint32_t id,
    tap::can::CanBus cB,
    TurretMCBCanComm* msgHandler,
    CanCommListenerFunc funcToCall)
    : CanRxListener(drivers, id, cB),
      msgHandler(msgHandler),
      funcToCall(funcToCall)
{
}

void TurretMCBCanComm::ImuRxHandler::processMessage(const modm::can::Message& message)
{
    (msgHandler->*funcToCall)(message);
}

void TurretMCBCanComm::TurretStatusRxHandler::processMessage(const modm::can::Message& message)
{
    (msgHandler->*funcToCall)(message);
}

}  // namespace aruwsrc::can
