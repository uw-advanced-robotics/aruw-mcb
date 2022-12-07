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

#include <cassert>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "aruwsrc/drivers.hpp"
#include "aruwsrc/util_macros.hpp"

using namespace tap::arch;
using namespace tap::communication::serial;
using namespace aruwsrc::serial;
using tap::arch::clock::getTimeMicroseconds;

VisionCoprocessor* VisionCoprocessor::visionCoprocessorInstance = nullptr;

#ifndef PLATFORM_HOSTED
MODM_ISR(EXTI0)
{
    // Currently the EXTI0 interrupt handler is only used by the time sync pin
    VisionCoprocessor::TimeSyncTriggerPin::acknowledgeExternalInterruptFlag();
    VisionCoprocessor::handleTimeSyncRequest();
}
#endif

VisionCoprocessor::VisionCoprocessor(aruwsrc::Drivers* drivers)
    : DJISerial(drivers, VISION_COPROCESSOR_RX_UART_PORT),
      risingEdgeTime(0),
      lastAimData(),
      odometryInterface(nullptr),
      turretOrientationInterfaces{}
{
#ifndef ENV_UNIT_TESTS
    // when testing it is OK to have multiple vision coprocessor instances, so this assertion
    // doesn't hold
    assert(visionCoprocessorInstance == nullptr);
#endif
    visionCoprocessorInstance = this;

    // Initialize all aim state to be invalid/unknown
    for (size_t i = 0; i < control::turret::NUM_TURRETS; i++)
    {
        this->lastAimData[i].hasTarget = 0;
        this->lastAimData[i].timestamp = 0;
    }
}

VisionCoprocessor::~VisionCoprocessor() { visionCoprocessorInstance = nullptr; }

void VisionCoprocessor::initializeCV()
{
#define DISABLE_TIME_SYNC_INTERRUPT
#if !defined(PLATFORM_HOSTED) && !defined(DISABLE_TIME_SYNC_INTERRUPT)
    // Set up the interrupt for the vision coprocessor sync handler
    VisionCoprocessor::TimeSyncTriggerPin::setInput(modm::platform::Gpio::InputType::PullDown);
    VisionCoprocessor::TimeSyncTriggerPin::enableExternalInterruptVector(0);
    VisionCoprocessor::TimeSyncTriggerPin::enableExternalInterrupt();
    VisionCoprocessor::TimeSyncTriggerPin::setInputTrigger(
        modm::platform::Gpio::InputTrigger::RisingEdge);
#endif

    cvOfflineTimeout.restart(TIME_OFFLINE_CV_AIM_DATA_MS);
#if defined(TARGET_HERO_CYCLONE)
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
            decodeToTurretAimData(completeMessage);
            return;
        }
        default:
            return;
    }
}

bool VisionCoprocessor::decodeToTurretAimData(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength == 0) {
        return false;
    } 
    uint8_t flagMask = ((1<<NUM_TYPES)-1);
    uint8_t header = message.data[0] | flagMask;
    int dataLength = 0;
    for (int i = 0; i < NUM_TYPES; ++i) {

        
    }

    /*
    if (message.header.dataLength != sizeof(lastAimData))
    {
        return false;
    }
    memcpy(lastAimData, &message.data, sizeof(lastAimData));
    return true;
    */
}

void VisionCoprocessor::sendMessage()
{
    sendOdometryData();
    sendRobotTypeData();
    sendTimeSyncMessage();
}

bool VisionCoprocessor::isCvOnline() const { return !cvOfflineTimeout.isExpired(); }

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
    assert(odometryInterface != nullptr);

    DJISerial::SerialMessage<sizeof(OdometryData)> odometryMessage;
    OdometryData* odometryData = reinterpret_cast<OdometryData*>(&odometryMessage.data);

    odometryMessage.messageType = CV_MESSAGE_TYPE_ODOMETRY_DATA;

    modm::Location2D<float> location = odometryInterface->getCurrentLocation2D();

    float pitch = modm::toRadian(drivers->mpu6500.getPitch());
    float roll = modm::toRadian(drivers->mpu6500.getRoll());
    // transform the pitch/roll from the chassis frame to the world frame
    tap::algorithms::rotateVector(&pitch, &roll, -location.getOrientation() - MCB_ROTATION_OFFSET);

    // chassis odometry
    odometryData->chassisOdometry.timestamp = getTimeMicroseconds();
    odometryData->chassisOdometry.xPos = location.getX();
    odometryData->chassisOdometry.yPos = location.getY();
    odometryData->chassisOdometry.zPos = 0.0f;
#if defined(ALL_SENTRIES)
    odometryData->chassisOdometry.pitch = 0;
    odometryData->chassisOdometry.roll = 0;
    odometryData->chassisOdometry.yaw = 0;
#else
    odometryData->chassisOdometry.pitch = pitch;
    odometryData->chassisOdometry.roll = roll;
    odometryData->chassisOdometry.yaw = location.getOrientation();
#endif

    // number of turrets
    odometryData->numTurrets = control::turret::NUM_TURRETS;

    // turret odometry
    for (size_t i = 0; i < MODM_ARRAY_SIZE(odometryData->turretOdometry); i++)
    {
        assert(turretOrientationInterfaces[i] != nullptr);
        odometryData->turretOdometry[i].timestamp =
            turretOrientationInterfaces[i]->getLastMeasurementTimeMicros();
        odometryData->turretOdometry[i].pitch = turretOrientationInterfaces[i]->getWorldPitch();
        odometryData->turretOdometry[i].yaw = turretOrientationInterfaces[i]->getWorldYaw();
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

void VisionCoprocessor::sendTimeSyncMessage()
{
    uint32_t newRisingEdgeTime = risingEdgeTime;

    if (prevRisingEdgeTime != newRisingEdgeTime)
    {
        prevRisingEdgeTime = newRisingEdgeTime;

        DJISerial::SerialMessage<sizeof(uint32_t) + sizeof(uint8_t)> timeSyncResponseMessage;

        timeSyncResponseMessage.messageType = CV_MESSAGE_TYPE_TIME_SYNC_RESP;

        *reinterpret_cast<uint32_t*>(timeSyncResponseMessage.data) = risingEdgeTime;
        *reinterpret_cast<uint8_t*>(timeSyncResponseMessage.data + sizeof(uint32_t)) = 0;
        timeSyncResponseMessage.setCRC16();

        drivers->uart.write(
            VISION_COPROCESSOR_TX_UART_PORT,
            reinterpret_cast<uint8_t*>(&timeSyncResponseMessage),
            sizeof(timeSyncResponseMessage));
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
