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

#include "tmotor_ak80_9.hpp"

#include "tap/drivers.hpp"

#ifdef PLATFORM_HOSTED
#include <iostream>

#include "tap/communication/tcp-server/json_messages.hpp"
#include "tap/communication/tcp-server/tcp_server.hpp"

#include "modm/architecture/interface/can_message.hpp"
#endif

namespace aruwsrc::control::motor
{
Tmotor_AK809::~Tmotor_AK809() {}

Tmotor_AK809::Tmotor_AK809(
    tap::Drivers* drivers,
    TMotorId desMotorIdentifier,
    tap::can::CanBus motorCanBus,
    bool isInverted,
    const char* name,
    int32_t encoderHomePosition)
    : CanRxListener(drivers, 0x2900 | static_cast<uint32_t>(desMotorIdentifier), motorCanBus),
      motorName(name),
      drivers(drivers),
      motorIdentifier(desMotorIdentifier),
      motorCanBus(motorCanBus),
      desiredOutput(0),
      temperature(0),
      torque(0),
      fault(TMotorFaultCode::FAULT_CODE_NONE),
      motorInverted(isInverted),
      internalEncoder(isInverted, GEAR_RATIO, encoderHomePosition)
{
    motorDisconnectTimeout.stop();
}

void Tmotor_AK809::initialize()
{
    attachSelfToRxHandler();
    sendPositionHomeGetMessage();
}

void Tmotor_AK809::resetEncoderValue() { return; }

void Tmotor_AK809::processMessage(const modm::can::Message& message)
{
    torque = static_cast<int16_t>(message.data[4] << 8 | message.data[5]);  // torque
    torque = motorInverted ? -torque : torque;
    temperature = static_cast<int8_t>(message.data[6]);  // temperature
    fault = static_cast<uint8_t>(message.data[7]);

    // restart disconnect timer, since you just received a message from the motor
    motorDisconnectTimeout.restart(MOTOR_DISCONNECT_TIME);

    this->internalEncoder.processMessage(message);
}

void Tmotor_AK809::setDesiredOutput(int32_t desiredOutput)
{
    int32_t invertedOutput = motorInverted ? -desiredOutput : desiredOutput;
    this->desiredOutput = tap::algorithms::limitVal<int32_t>(invertedOutput, -60000, 60000);
}

bool Tmotor_AK809::isMotorOnline() const
{
    /*
     * motor online if the disconnect timout has not expired (if it received message but
     * somehow got disconnected) and the timeout hasn't been stopped (initially, the timeout
     * is stopped)
     */
    if (!motorDisconnectTimeout.isExpired() && !motorDisconnectTimeout.isStopped())
    {
        return true;
    }
    else
    {
        sendPositionHomeGetMessage();
        return false;
    }
}

bool Tmotor_AK809::sendCanMessage()
{
    modm::can::Message message(
        (uint32_t)(motorIdentifier) |
            ((uint32_t)0x01 << 8),  // the 01 in LSByte 2 sets motor to current mode
        CAN_TMOTOR_MESSAGE_SEND_LENGTH);
    message.setExtended();
    message.setRemoteTransmitRequest(false);

    message.data[0] = desiredOutput >> 24;
    message.data[1] = desiredOutput >> 16;
    message.data[2] = desiredOutput >> 8;
    message.data[3] = desiredOutput;

    bool messageSuccess = true;

    if (drivers->can.isReadyToSend(tap::can::CanBus::CAN_BUS1) &&
        motorCanBus == tap::can::CanBus::CAN_BUS1)
    {
        messageSuccess &= drivers->can.sendMessage(tap::can::CanBus::CAN_BUS1, message);
    }
    if (drivers->can.isReadyToSend(tap::can::CanBus::CAN_BUS2) &&
        motorCanBus == tap::can::CanBus::CAN_BUS2)
    {
        messageSuccess &= drivers->can.sendMessage(tap::can::CanBus::CAN_BUS2, message);
    }
    return messageSuccess;
}

bool Tmotor_AK809::sendPositionHomeResetMessage() const
{
    modm::can::Message homingMessage(
        (uint32_t)(motorIdentifier) |
            ((uint32_t)0x05 << 8),  // the 05 in LSByte 2 sets motor to pos home mode
        8);
    homingMessage.setExtended();
    homingMessage.data[0] = 0x1;  // sets the permanent origin
    return drivers->can.sendMessage(motorCanBus, homingMessage);
}

bool Tmotor_AK809::sendPositionHomeGetMessage() const
{
    modm::can::Message homingMessage(
        (uint32_t)(motorIdentifier) |
            ((uint32_t)0x05 << 8),  // the 05 in LSByte 2 sets motor to pos home mode
        8);                         // data length is 8 as per the protocol
    homingMessage.setExtended();
    homingMessage.data[0] = 0x2;  // gets the permanent origin
    return drivers->can.sendMessage(motorCanBus, homingMessage);
}

// getter functions
int16_t Tmotor_AK809::getOutputDesired() const { return desiredOutput; }

uint32_t Tmotor_AK809::getMotorIdentifier() const { return motorIdentifier; }

int8_t Tmotor_AK809::getTemperature() const { return temperature; }

int16_t Tmotor_AK809::getTorque() const { return torque; }

bool Tmotor_AK809::isMotorInverted() const { return motorInverted; }

tap::can::CanBus Tmotor_AK809::getCanBus() const { return motorCanBus; }

const char* Tmotor_AK809::getName() const { return motorName; }

}  // namespace aruwsrc::control::motor
