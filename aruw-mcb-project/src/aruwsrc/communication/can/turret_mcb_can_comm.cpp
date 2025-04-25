/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "modm/architecture/interface/can.hpp"

namespace aruwsrc::can
{
TurretMCBCanComm::TurretMCBCanComm(tap::Drivers* drivers, tap::can::CanBus canBus)
    : canBus(canBus),
      drivers(drivers),
      currProcessingImuData{},
      lastCompleteImuData{},
      yawRevolutions(0),
      pitchRevolutions(0),
      xAxisMessageHandler(
          drivers,
          X_AXIS_RX_CAN_ID,
          canBus,
          this,
          &TurretMCBCanComm::handleXAxisMessage),
      yAxisMessageHandler(
          drivers,
          Y_AXIS_RX_CAN_ID,
          canBus,
          this,
          &TurretMCBCanComm::handleYAxisMessage),
      zAxisMessageHandler(
          drivers,
          Z_AXIS_RX_CAN_ID,
          canBus,
          this,
          &TurretMCBCanComm::handleZAxisMessage),
      turretStatusRxHandler(
          drivers,
          TURRET_STATUS_RX_CAN_ID,
          canBus,
          this,
          &TurretMCBCanComm::handleTurretMessage)
{
}

void TurretMCBCanComm::init()
{
    xAxisMessageHandler.attachSelfToRxHandler();
    yAxisMessageHandler.attachSelfToRxHandler();
    zAxisMessageHandler.attachSelfToRxHandler();
    turretStatusRxHandler.attachSelfToRxHandler();
}

void TurretMCBCanComm::sendData()
{
    if (requestCalibration)
    {
        modm::can::Message txMsg(TURRET_MCB_TX_CAN_ID, 1);
        txMsg.setExtended(false);
        drivers->can.sendMessage(canBus, txMsg);

        yawRevolutions = 0;
        pitchRevolutions = 0;

        requestCalibration = false;
    }

    if (!isConnected())
    {
        yawRevolutions = 0;
        pitchRevolutions = 0;
    }
}

void TurretMCBCanComm::handleXAxisMessage(const modm::can::Message& message)
{
    // Update light to indicate IMU message received and turret controller running.
    imuMessageReceivedLEDBlinkCounter = (imuMessageReceivedLEDBlinkCounter + 1) % 100;
    drivers->leds.set(tap::gpio::Leds::Green, imuMessageReceivedLEDBlinkCounter > 50);
    imuConnectedTimeout.restart(DISCONNECT_TIMEOUT_PERIOD);

    const AxisMessageData* xAxisMessage = reinterpret_cast<const AxisMessageData*>(message.data);

    currProcessingImuData.roll =
        static_cast<float>(xAxisMessage->angleFixedPoint) * ANGLE_FIXED_POINT_PRECISION;
    currProcessingImuData.rollVelocity = xAxisMessage->angleAngularVelocityRaw;
    currProcessingImuData.xAcceleration =
        static_cast<float>(xAxisMessage->linearAcceleration) * CMPS2_TO_MPS2;

    /**
     * Since this is the first axis data received for a full IMU message,
     * set the IMU sequence to the current one to check the other axis
     * data against.
     *
     * Set the timestamp as well since this is the closest we'll get to
     * when the data was measured on the turret mcb.
     */

    currProcessingImuData.seq = xAxisMessage->seq;
    currProcessingImuData.turretDataTimestamp = tap::arch::clock::getTimeMicroseconds();
}

void TurretMCBCanComm::handleYAxisMessage(const modm::can::Message& message)
{
    const AxisMessageData* yAxisMessage = reinterpret_cast<const AxisMessageData*>(message.data);

    if (yAxisMessage->seq != currProcessingImuData.seq)
    {
        RAISE_ERROR(drivers, "seq # mismatch when handling y-axis data");
        return;
    }

    currProcessingImuData.pitch =
        static_cast<float>(yAxisMessage->angleFixedPoint) * ANGLE_FIXED_POINT_PRECISION;
    currProcessingImuData.pitchVelocity = yAxisMessage->angleAngularVelocityRaw;
    currProcessingImuData.yAcceleration =
        static_cast<float>(yAxisMessage->linearAcceleration) * CMPS2_TO_MPS2;
}

void TurretMCBCanComm::handleZAxisMessage(const modm::can::Message& message)
{
    const AxisMessageData* zAxisMessage = reinterpret_cast<const AxisMessageData*>(message.data);

    if (zAxisMessage->seq != currProcessingImuData.seq)
    {
        RAISE_ERROR(drivers, "seq # mismatch when handling z-axis data");
        return;
    }

    currProcessingImuData.yaw =
        static_cast<float>(zAxisMessage->angleFixedPoint) * ANGLE_FIXED_POINT_PRECISION;
    currProcessingImuData.yawVelocity = zAxisMessage->angleAngularVelocityRaw;
    currProcessingImuData.zAcceleration =
        static_cast<float>(zAxisMessage->linearAcceleration) * CMPS2_TO_MPS2;

    /**
     * Since this is the last axis data received for a full IMU data message,
     * apply post-processing and update the lastCompleteImuData to the processed data.
     * Also call the callback function if one exists.
     */

    updateRevolutionCounter(currProcessingImuData.roll, lastCompleteImuData.roll, rollRevolutions);

    updateRevolutionCounter(
        currProcessingImuData.pitch,
        lastCompleteImuData.pitch,
        pitchRevolutions);

    updateRevolutionCounter(currProcessingImuData.yaw, lastCompleteImuData.yaw, yawRevolutions);

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

TurretMCBCanComm::TurretMcbRxHandler::TurretMcbRxHandler(
    tap::Drivers* drivers,
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
