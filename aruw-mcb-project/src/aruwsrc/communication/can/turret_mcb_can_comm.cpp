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
#include "tap/errors/create_errors.hpp"

#include "aruwsrc/constants.hpp"
#include "aruwsrc/drivers.hpp"
#include "modm/architecture/interface/can.hpp"

namespace aruwsrc::can
{
TurretMCBCanComm::TurretMCBCanComm(aruwsrc::Drivers* drivers)
    : drivers(drivers),
      yawAngleGyroMessageHandler(
          drivers,
          YAW_RX_CAN_ID,
          TURRET_MCB_CAN_BUS,
          this,
          &TurretMCBCanComm::handleYawAngleGyroMessage),
      pitchAngleGyroMessageHandler(
          drivers,
          PITCH_RX_CAN_ID,
          TURRET_MCB_CAN_BUS,
          this,
          &TurretMCBCanComm::handlePitchAngleGyroMessage),
      turretStatusRxHandler(
          drivers,
          TURRET_STATUS_RX_CAN_ID,
          TURRET_MCB_CAN_BUS,
          this,
          &TurretMCBCanComm::handleTurretMessage),
      timeSynchronizationRxHandler(
          drivers,
          SYNC_RX_CAN_ID,
          TURRET_MCB_CAN_BUS,
          this,
          &TurretMCBCanComm::handleTimeSynchronizationRequest),
      txCommandMsgBitmask(),
      sendMcbDataTimer(SEND_MCB_DATA_TIMEOUT)
{
}

void TurretMCBCanComm::init()
{
    yawAngleGyroMessageHandler.attachSelfToRxHandler();
    pitchAngleGyroMessageHandler.attachSelfToRxHandler();
    turretStatusRxHandler.attachSelfToRxHandler();
    timeSynchronizationRxHandler.attachSelfToRxHandler();
}

void TurretMCBCanComm::sendData()
{
    if (sendMcbDataTimer.execute())
    {
        modm::can::Message txMsg(TURRET_MCB_TX_CAN_ID, 1);
        txMsg.setExtended(false);
        txMsg.data[0] = txCommandMsgBitmask.value;
        drivers->can.sendMessage(tap::can::CanBus::CAN_BUS1, txMsg);

        // set this calibrate flag to false so the calibrate command is only sent once
        txCommandMsgBitmask.reset(TxCommandMsgBitmask::RECALIBRATE_IMU);
    }
}

void TurretMCBCanComm::handleYawAngleGyroMessage(const modm::can::Message& message)
{
    // Update light to indicate IMU message received and turret controller running.
    imuMessageReceivedLEDBlinkCounter = (imuMessageReceivedLEDBlinkCounter + 1) % 100;
    drivers->leds.set(tap::gpio::Leds::Green, imuMessageReceivedLEDBlinkCounter > 50);
    imuConnectedTimeout.restart(DISCONNECT_TIMEOUT_PERIOD);

    const AngleMessageData* angleMessage = reinterpret_cast<const AngleMessageData*>(message.data);

    currProcessingImuData.yaw =
        static_cast<float>(angleMessage->angleFixedPoint) * ANGLE_FIXED_POINT_PRECISION;
    currProcessingImuData.rawYawVelocity = angleMessage->angleAngularVelocityRaw;
    currProcessingImuData.seq = angleMessage->seq;
    // clear top 16 bits
    currProcessingImuData.turretDataTimestamp &= 0xffff;
    // fill in top 16 bits
    currProcessingImuData.turretDataTimestamp |= static_cast<uint32_t>(angleMessage->timestamp)
                                                 << 16;
}

void TurretMCBCanComm::handlePitchAngleGyroMessage(const modm::can::Message& message)
{
    const AngleMessageData* angleMessage = reinterpret_cast<const AngleMessageData*>(message.data);

    // if seq # doesn't match, raise error (means some data was lost)
    if (angleMessage->seq != currProcessingImuData.seq)
    {
        RAISE_ERROR(drivers, "seq # mismatch when handling pitch angle data");
        return;
    }

    currProcessingImuData.pitch =
        static_cast<float>(angleMessage->angleFixedPoint) * ANGLE_FIXED_POINT_PRECISION;
    currProcessingImuData.rawPitchVelocity = angleMessage->angleAngularVelocityRaw;
    // clear bottom 16 bits
    currProcessingImuData.turretDataTimestamp &= 0xffff0000;
    // fill in bottom 16 bits
    currProcessingImuData.turretDataTimestamp |= static_cast<uint32_t>(angleMessage->timestamp);

    lastCompleteImuData = currProcessingImuData;

    if (imuDataReceivedCallbackFunc != nullptr)
    {
        imuDataReceivedCallbackFunc();
    }
}

void TurretMCBCanComm::handleTurretMessage(const modm::can::Message& message)
{
    limitSwitchDepressed = message.data[0] & 0b1;
}

void TurretMCBCanComm::handleTimeSynchronizationRequest(const modm::can::Message&)
{
    modm::can::Message syncResponseMessage(SYNC_TX_CAN_ID, 4);
    syncResponseMessage.setExtended(false);
    *reinterpret_cast<uint32_t*>(syncResponseMessage.data) =
        tap::arch::clock::getTimeMicroseconds();
    drivers->can.sendMessage(TURRET_MCB_CAN_BUS, syncResponseMessage);
}

TurretMCBCanComm::TurretMcbRxHandler::TurretMcbRxHandler(
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

void TurretMCBCanComm::TurretMcbRxHandler::processMessage(const modm::can::Message& message)
{
    (msgHandler->*funcToCall)(message);
}

}  // namespace aruwsrc::can
