/*
 * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 */

#include "tmotor_ak80-9.hpp"

#include "tap/drivers.hpp"

#ifdef PLATFORM_HOSTED
#include <iostream>

#include "tap/communication/tcp-server/json_messages.hpp"
#include "tap/communication/tcp-server/tcp_server.hpp"

#include "modm/architecture/interface/can_message.hpp"
#endif

namespace aruwsrc
{
namespace motor
{
Tmotor_AK809::~Tmotor_AK809() {}

Tmotor_AK809::Tmotor_AK809(
    tap::Drivers* drivers,
    aruwsrc::motor::TMotorId desMotorIdentifier,
    tap::can::CanBus motorCanBus,
    bool isInverted,
    const char* name,
    int64_t encoderRevolutions)
    : CanRxListener(drivers, static_cast<uint32_t>(desMotorIdentifier), motorCanBus),
      motorName(name),
      drivers(drivers),
      motorIdentifier(desMotorIdentifier),
      motorCanBus(motorCanBus),
      desiredOutput(0),
      shaftRPM(0),
      temperature(0),
      torque(0),
      motorInverted(isInverted),
      encoderRevolutions(encoderRevolutions)
{
    motorDisconnectTimeout.stop();
}

void Tmotor_AK809::initialize()
{
    attachSelfToRxHandler();
    sendPositionHomeResetMessage();
    sendPositionHomeGetMessage();
}

void Tmotor_AK809::processMessage(const modm::can::Message& message)
{
    if ((message.getIdentifier() - 0x2900) !=
        Tmotor_AK809::getMotorIdentifier())  // magic offset for return messages
    {
        return;
    }
    /***
     * WARNING! The Ak80-9 initializes it's encoder position to 1750 on boot.
     */
    encoderPosition =
        static_cast<int16_t>(message.data[0] << 8 | message.data[1]);  // position, mDeg
    /***
     * WARNING! the AK80-9 outputs position in the range [-32000, 32000]
     * If the output shaft rotates more than this, then the output value will saturate! The motor
     * continues to track it's position but you will not get the values over CAN. Try to not
     * overrotate the motor.
     */
    shaftRPM = static_cast<int16_t>(message.data[2] << 8 | message.data[3]);  // rpm
    shaftRPM = motorInverted ? -shaftRPM : shaftRPM;
    torque = static_cast<int16_t>(message.data[4] << 8 | message.data[5]);  // current in mA
    torque = motorInverted ? -torque : torque;
    temperature = static_cast<int8_t>(message.data[6]);  // temperature

    // restart disconnect timer, since you just received a message from the motor
    motorDisconnectTimeout.restart(MOTOR_DISCONNECT_TIME);

    // invert motor if necessary
    encoderPosition = motorInverted ? encoderPosition : -encoderPosition;
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
        sendPositionHomeResetMessage();
        return false;
    }
}

bool Tmotor_AK809::sendCanMessage()
{
    debug1 += 1;
    modm::can::Message message(
        (uint32_t)(motorIdentifier) |
            ((uint32_t)0x01 << 8),  // the 01 in LSByte 2 sets motor to current mode
        CAN_TMOTOR_MESSAGE_SEND_LENGTH,
        0,
        true);
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
        debug2 += 1;
    }
    return messageSuccess;
}

bool Tmotor_AK809::sendPositionHomeResetMessage() const
{
    modm::can::Message homingMessage(
        (uint32_t)(motorIdentifier) |
            ((uint32_t)0x05 << 8),  // the 05 in LSByte 2 sets motor to pos home mode
        8,                          // data length is 8 as per the protocol
        0,
        true);
    homingMessage.data[0] = 0x1;  // sets the permanent origin
    return drivers->can.sendMessage(motorCanBus, homingMessage);
}

bool Tmotor_AK809::sendPositionHomeGetMessage() const
{
    modm::can::Message homingMessage(
        (uint32_t)(motorIdentifier) |
            ((uint32_t)0x05 << 8),  // the 05 in LSByte 2 sets motor to pos home mode
        8,                          // data length is 8 as per the protocol
        0,
        true);
    homingMessage.data[0] = 0x2;  // gets the permanent origin
    return drivers->can.sendMessage(motorCanBus, homingMessage);
}

// getter functions
int16_t Tmotor_AK809::getOutputDesired() const { return desiredOutput; }

uint32_t Tmotor_AK809::getMotorIdentifier() const { return motorIdentifier; }

int8_t Tmotor_AK809::getTemperature() const { return temperature; }

int16_t Tmotor_AK809::getTorque() const { return torque; }

int16_t Tmotor_AK809::getShaftRPM() const { return shaftRPM; }

bool Tmotor_AK809::isMotorInverted() const { return motorInverted; }

tap::can::CanBus Tmotor_AK809::getCanBus() const { return motorCanBus; }

const char* Tmotor_AK809::getName() const { return motorName; }

int64_t Tmotor_AK809::getEncoderUnwrapped() const { return static_cast<int64_t>(encoderPosition); }

uint16_t Tmotor_AK809::getEncoderWrapped() const
{
    if (encoderPosition >= 0)
    {
        return encoderPosition % 3600;
    }
    else
    {
        return 3600 + encoderPosition % 3600;
    }
}

}  // namespace motor

}  // namespace aruwsrc
