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

#include <algorithm>
#include <cmath>
#include <limits>

#include "tap/architecture/endianness_wrappers.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "modm/architecture/interface/can.hpp"

namespace aruwsrc::communication::can
{
namespace
{
constexpr float ROTATION_COMPONENT_SCALE = 1000.0f;
constexpr float TRANSLATION_COMPONENT_SCALE = 10.0f;
constexpr uint8_t IMU_MOUNTING_SYNC_BURST_COUNT = 3;
constexpr uint8_t CALIBRATION_SAMPLES_SYNC_BURST_COUNT = 3;

inline int16_t quantizeTransformComponent(float value, float scale)
{
    return static_cast<int16_t>(std::clamp(
        std::lround(value * scale),
        static_cast<long>(std::numeric_limits<int16_t>::min()),
        static_cast<long>(std::numeric_limits<int16_t>::max())));
}
}  // namespace

TurretMCBCanComm::TurretMCBCanComm(tap::Drivers* drivers, tap::can::CanBus canBus)
    : AbstractIMU(),
      canBus(canBus),
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
          &TurretMCBCanComm::handleTurretMessage),
      timeSynchronizationRxHandler(
          drivers,
          SYNC_RX_CAN_ID,
          canBus,
          this,
          &TurretMCBCanComm::handleTimeSynchronizationRequest),
      calibrationSamplesRequestRxHandler(
          drivers,
          CALIBRATION_SAMPLES_REQUEST_RX_CAN_ID,
          canBus,
          this,
          &TurretMCBCanComm::handleCalibrationSamplesRequest),
      imuMountingRequestRxHandler(
          drivers,
          IMU_MOUNTING_REQUEST_RX_CAN_ID,
          canBus,
          this,
          &TurretMCBCanComm::handleImuMountingTransformRequest),
      txCommandMsgBitmask(),
      sendMcbDataTimer(SEND_MCB_DATA_TIMEOUT)
{
    imuState = ImuState::IMU_NOT_CONNECTED;
}

void TurretMCBCanComm::init()
{
    xAxisMessageHandler.attachSelfToRxHandler();
    yAxisMessageHandler.attachSelfToRxHandler();
    zAxisMessageHandler.attachSelfToRxHandler();
    turretStatusRxHandler.attachSelfToRxHandler();
    timeSynchronizationRxHandler.attachSelfToRxHandler();
    calibrationSamplesRequestRxHandler.attachSelfToRxHandler();
    imuMountingRequestRxHandler.attachSelfToRxHandler();
}

void TurretMCBCanComm::initialize(float, float, float)
{
    imuData = {};
    currProcessingImuData = {};
    lastCompleteImuData = {};
    prevIMUDataReceivedTime = 0;
    yawRevolutions = 0;
    pitchRevolutions = 0;
    rollRevolutions = 0;
    imuState = ImuState::IMU_NOT_CONNECTED;
}

void TurretMCBCanComm::periodicIMUUpdate()
{
    // IMU fusion/calibration is performed on the turret MCB side.
}

void TurretMCBCanComm::sendData()
{
    if (sendMcbDataTimer.execute())
    {
        modm::can::Message txMsg(TURRET_MCB_TX_CAN_ID, 1);
        txMsg.setExtended(false);
        txMsg.data[0] = txCommandMsgBitmask.value;
        drivers->can.sendMessage(canBus, txMsg);

        if (txCommandMsgBitmask.any(TxCommandMsgBitmask::RECALIBRATE_IMU))
        {
            yawRevolutions = 0;
            pitchRevolutions = 0;
        }

        // set this calibrate flag to false so the calibrate command is only sent once
        txCommandMsgBitmask.reset(TxCommandMsgBitmask::RECALIBRATE_IMU);
    }

    if (!isConnected())
    {
        imuData = {};
        currProcessingImuData = {};
        lastCompleteImuData = {};
        prevIMUDataReceivedTime = 0;
        yawRevolutions = 0;
        pitchRevolutions = 0;
        rollRevolutions = 0;
        imuState = ImuState::IMU_NOT_CONNECTED;
    }

    sendImuMountingTransformSync();
    sendCalibrationSamplesSync();
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
    currProcessingImuData.rawRollVelocity = xAxisMessage->angleAngularVelocityRaw;
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
    currProcessingImuData.rawPitchVelocity = yAxisMessage->angleAngularVelocityRaw;
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
    currProcessingImuData.rawYawVelocity = zAxisMessage->angleAngularVelocityRaw;
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
    imuData.accG = tap::algorithms::transforms::Vector(
        lastCompleteImuData.xAcceleration,
        lastCompleteImuData.yAcceleration,
        lastCompleteImuData.zAcceleration);
    imuData.gyroRadPerSec = tap::algorithms::transforms::Vector(
        static_cast<float>(lastCompleteImuData.rawRollVelocity) * IMU_SCALING_FACTOR,
        static_cast<float>(lastCompleteImuData.rawPitchVelocity) * IMU_SCALING_FACTOR,
        static_cast<float>(lastCompleteImuData.rawYawVelocity) * IMU_SCALING_FACTOR);
    // imuData.accRaw = imuData.accG;
    // imuData.gyroRaw = imuData.gyroRadPerSec;
    prevIMUDataReceivedTime = lastCompleteImuData.turretDataTimestamp;

    if (imuDataReceivedCallbackFunc != nullptr)
    {
        imuDataReceivedCallbackFunc();
    }
}

void TurretMCBCanComm::handleTurretMessage(const modm::can::Message& message)
{
    if (message.getLength() >= sizeof(TurretStatusMessageData))
    {
        const TurretStatusMessageData* status =
            reinterpret_cast<const TurretStatusMessageData*>(message.data);
        limitSwitchDepressed = status->statusBitmask & 0b1;

        const uint8_t stateRaw = status->imuState;
        if (stateRaw <= static_cast<uint8_t>(ImuState::IMU_CALIBRATED))
        {
            imuState = static_cast<ImuState>(stateRaw);
        }
        else
        {
            imuState = ImuState::IMU_NOT_CONNECTED;
        }

        const float temperature = static_cast<float>(status->temperatureCentiC) * 0.01f;
        lastCompleteImuData.temperature = temperature;
        currProcessingImuData.temperature = temperature;
        imuData.temperature = temperature;
    }
    else
    {
        // Legacy status payload: only limit switch bit.
        limitSwitchDepressed = message.data[0] & 0b1;
    }
}

void TurretMCBCanComm::handleTimeSynchronizationRequest(const modm::can::Message&)
{
    modm::can::Message syncResponseMessage(SYNC_TX_CAN_ID, 4);
    syncResponseMessage.setExtended(false);
    *reinterpret_cast<uint32_t*>(syncResponseMessage.data) =
        tap::arch::clock::getTimeMicroseconds();
    drivers->can.sendMessage(canBus, syncResponseMessage);
}

void TurretMCBCanComm::handleCalibrationSamplesRequest(const modm::can::Message&)
{
    queueCalibrationSamplesSync();
}

void TurretMCBCanComm::setImuMountingTransforms(
    const tap::algorithms::transforms::Transform& bmi088MountingTransform,
    const tap::algorithms::transforms::Transform& ism330MountingTransform)
{
    clearImuMountingTransforms();
    setImuMountingTransform(RemoteImuType::BMI088, bmi088MountingTransform);
    setImuMountingTransform(RemoteImuType::ISM330, ism330MountingTransform);
}

void TurretMCBCanComm::clearImuMountingTransforms() { hasRemoteImuMountingTransform.fill(false); }

void TurretMCBCanComm::setImuMountingTransform(
    RemoteImuType imuType,
    const tap::algorithms::transforms::Transform& mountingTransform)
{
    const size_t imuIndex = static_cast<size_t>(imuType);
    if (imuIndex >= remoteImuMountingTransforms.size())
    {
        return;
    }

    remoteImuMountingTransforms[imuIndex] = mountingTransform;
    hasRemoteImuMountingTransform[imuIndex] = true;
}

void TurretMCBCanComm::queueImuMountingTransformSync()
{
    imuMountingSyncBurstsRemaining = IMU_MOUNTING_SYNC_BURST_COUNT;
}

void TurretMCBCanComm::queueCalibrationSamplesSync()
{
    calibrationSamplesSyncBurstsRemaining = CALIBRATION_SAMPLES_SYNC_BURST_COUNT;
}

void TurretMCBCanComm::handleImuMountingTransformRequest(const modm::can::Message&)
{
    if (hasAnyImuMountingTransformsConfigured())
    {
        queueImuMountingTransformSync();
    }
}

bool TurretMCBCanComm::sendImuMountingTransformSyncMessage(
    RemoteImuType imuType,
    TransformMessagePart part,
    const tap::algorithms::transforms::Transform& transform)
{
    if (!drivers->can.isReadyToSend(canBus))
    {
        return false;
    }

    modm::can::Message msg(IMU_MOUNTING_TX_CAN_ID, sizeof(ImuMountingTransformMessageData));
    msg.setExtended(false);
    auto* payload = reinterpret_cast<ImuMountingTransformMessageData*>(msg.data);
    payload->imuType = static_cast<uint8_t>(imuType);
    payload->part = static_cast<uint8_t>(part);

    if (part == TransformMessagePart::TRANSLATION)
    {
        const auto translation = transform.getTranslation();
        payload->componentA =
            quantizeTransformComponent(translation.x(), TRANSLATION_COMPONENT_SCALE);
        payload->componentB =
            quantizeTransformComponent(translation.y(), TRANSLATION_COMPONENT_SCALE);
        payload->componentC =
            quantizeTransformComponent(translation.z(), TRANSLATION_COMPONENT_SCALE);
    }
    else
    {
        payload->componentA =
            quantizeTransformComponent(transform.getRoll(), ROTATION_COMPONENT_SCALE);
        payload->componentB =
            quantizeTransformComponent(transform.getPitch(), ROTATION_COMPONENT_SCALE);
        payload->componentC =
            quantizeTransformComponent(transform.getYaw(), ROTATION_COMPONENT_SCALE);
    }

    drivers->can.sendMessage(canBus, msg);
    return true;
}

bool TurretMCBCanComm::hasAnyImuMountingTransformsConfigured() const
{
    for (bool hasTransform : hasRemoteImuMountingTransform)
    {
        if (hasTransform)
        {
            return true;
        }
    }
    return false;
}

void TurretMCBCanComm::sendImuMountingTransformSync()
{
    if (imuMountingSyncBurstsRemaining == 0 || !hasAnyImuMountingTransformsConfigured())
    {
        return;
    }

    bool burstSuccess = true;
    bool sentAny = false;
    for (size_t i = 0; i < remoteImuMountingTransforms.size(); i++)
    {
        if (!hasRemoteImuMountingTransform[i])
        {
            continue;
        }

        sentAny = true;
        const auto imuType = static_cast<RemoteImuType>(i);
        const auto& transform = remoteImuMountingTransforms[i];

        const bool sentTranslation = sendImuMountingTransformSyncMessage(
            imuType,
            TransformMessagePart::TRANSLATION,
            transform);
        const bool sentRotation =
            sendImuMountingTransformSyncMessage(imuType, TransformMessagePart::ROTATION, transform);
        burstSuccess &= sentTranslation && sentRotation;
    }

    if (sentAny && burstSuccess)
    {
        imuMountingSyncBurstsRemaining--;
    }
}

void TurretMCBCanComm::sendCalibrationSamplesSync()
{
    if (calibrationSamplesSyncBurstsRemaining == 0 || !drivers->can.isReadyToSend(canBus))
    {
        return;
    }

    modm::can::Message msg(CALIBRATION_SAMPLES_TX_CAN_ID, sizeof(CalibrationSamplesMessageData));
    msg.setExtended(false);
    auto* payload = reinterpret_cast<CalibrationSamplesMessageData*>(msg.data);
    payload->samples = remoteCalibrationSampleCount;
    drivers->can.sendMessage(canBus, msg);
    calibrationSamplesSyncBurstsRemaining--;
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

}  // namespace aruwsrc::communication::can
