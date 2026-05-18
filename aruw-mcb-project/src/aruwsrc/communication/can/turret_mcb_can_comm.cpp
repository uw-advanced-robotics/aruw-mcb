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

#include <cmath>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/endianness_wrappers.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "modm/architecture/interface/can.hpp"

namespace aruwsrc::communication::can
{
namespace
{
template <typename T>
inline T quantizeTransformComponent(float value, float scale)
{
    return static_cast<T>(round(tap::algorithms::limitVal<float>(
        value * scale,
        std::numeric_limits<T>::min(),
        std::numeric_limits<T>::max())));
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
      rollRevolutions(0),
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
        // set this calibrate flag to false after switching state
        if (imuState == ImuState::IMU_CALIBRATING)
        {
            txCommandMsgBitmask.reset(TxCommandMsgBitmask::RECALIBRATE_IMU);
        }

        modm::can::Message txMsg(TURRET_MCB_TX_CAN_ID, 1);
        txMsg.setExtended(false);
        memset(txMsg.data, 0, sizeof(txMsg.data));
        txMsg.data[0] = txCommandMsgBitmask.value;
        drivers->can.sendMessage(canBus, txMsg);

        if (txCommandMsgBitmask.any(TxCommandMsgBitmask::RECALIBRATE_IMU))
        {
            yawRevolutions = 0;
            pitchRevolutions = 0;
            rollRevolutions = 0;
        }
    }

    if (!isConnected())
    {
        imuData = {};
        currProcessingImuData = {};
        yawRevolutions = 0;
        pitchRevolutions = 0;
        rollRevolutions = 0;
        imuState = ImuState::IMU_NOT_CONNECTED;
    }
    if (imuMountingTransformQueued)
    {
        sendImuMountingTransformSync();
    }
    if (calibrationSamplesSyncQueued)
    {
        sendCalibrationSamplesSync();
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
    prevIMUDataReceivedTime = lastCompleteImuData.turretDataTimestamp;

    if (imuDataReceivedCallbackFunc != nullptr)
    {
        imuDataReceivedCallbackFunc();
    }
}

void TurretMCBCanComm::handleTurretMessage(const modm::can::Message& message)
{
    // Status frames are a heartbeat and should keep the remote IMU marked connected,
    // even when axis packets pause during calibration/transitions.
    imuConnectedTimeout.restart(DISCONNECT_TIMEOUT_PERIOD);
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
    const tap::algorithms::transforms::Transform& turretToBmi088,
    const tap::algorithms::transforms::Transform& turretToIsm330)
{
    clearHasImuMountingTransforms();
    setImuMountingTransform(RemoteImuType::BMI088, turretToBmi088);
    setImuMountingTransform(RemoteImuType::ISM330, turretToIsm330);
}

void TurretMCBCanComm::clearHasImuMountingTransforms()
{
    hasRemoteImuMountingTransform.fill(false);
}

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

void TurretMCBCanComm::queueImuMountingTransformSync() { imuMountingTransformQueued = true; }

void TurretMCBCanComm::queueCalibrationSamplesSync() { calibrationSamplesSyncQueued = true; }

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

    if (part == TransformMessagePart::TRANSLATION)
    {
        modm::can::Message msg(
            IMU_MOUNTING_TX_CAN_ID,
            sizeof(ImuMountingTransformMessageData<TranslationQuantType>));
        msg.setExtended(false);
        auto* payload =
            reinterpret_cast<ImuMountingTransformMessageData<TranslationQuantType>*>(msg.data);
        payload->imuType = static_cast<uint8_t>(imuType);
        payload->part = static_cast<uint8_t>(part);

        const auto translation = transform.getTranslation();
        payload->componentA = quantizeTransformComponent<TranslationQuantType>(
            translation.x(),
            TRANSLATION_COMPONENT_SCALE);
        payload->componentB = quantizeTransformComponent<TranslationQuantType>(
            translation.y(),
            TRANSLATION_COMPONENT_SCALE);
        payload->componentC = quantizeTransformComponent<TranslationQuantType>(
            translation.z(),
            TRANSLATION_COMPONENT_SCALE);
        drivers->can.sendMessage(canBus, msg);
    }
    else
    {
        modm::can::Message msg(
            IMU_MOUNTING_TX_CAN_ID,
            sizeof(ImuMountingTransformMessageData<RotationQuantType>));
        msg.setExtended(false);
        auto* payload =
            reinterpret_cast<ImuMountingTransformMessageData<RotationQuantType>*>(msg.data);
        payload->imuType = static_cast<uint8_t>(imuType);
        payload->part = static_cast<uint8_t>(part);

        // Make positive
        auto wrapPositive = [](float angle) { return angle < 0.0f ? angle + M_TWOPI : angle; };

        const float roll = wrapPositive(transform.getRoll());
        const float pitch = wrapPositive(transform.getPitch());
        const float yaw = wrapPositive(transform.getYaw());

        payload->componentA =
            quantizeTransformComponent<RotationQuantType>(roll, ROTATION_COMPONENT_SCALE);
        payload->componentB =
            quantizeTransformComponent<RotationQuantType>(pitch, ROTATION_COMPONENT_SCALE);
        payload->componentC =
            quantizeTransformComponent<RotationQuantType>(yaw, ROTATION_COMPONENT_SCALE);

        drivers->can.sendMessage(canBus, msg);
    }

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
    if (!hasAnyImuMountingTransformsConfigured())
    {
        return;
    }

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

        sendImuMountingTransformSyncMessage(imuType, TransformMessagePart::TRANSLATION, transform);
        sendImuMountingTransformSyncMessage(imuType, TransformMessagePart::ROTATION, transform);
    }
    if (sentAny)
    {
        imuMountingTransformQueued = false;
    }
}

void TurretMCBCanComm::sendCalibrationSamplesSync()
{
    if (!drivers->can.isReadyToSend(canBus))
    {
        return;
    }

    modm::can::Message msg(CALIBRATION_SAMPLES_TX_CAN_ID, sizeof(CalibrationSamplesMessageData));
    msg.setExtended(false);
    auto* payload = reinterpret_cast<CalibrationSamplesMessageData*>(msg.data);
    payload->samples = remoteCalibrationSampleCount;
    drivers->can.sendMessage(canBus, msg);
    calibrationSamplesSyncQueued = false;
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
