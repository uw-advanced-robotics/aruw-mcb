/*
 * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 */

#include "tmotor_ak80-9.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "aruwsrc/drivers.hpp"

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
Tmotor_AK809::~Tmotor_AK809() { drivers->tMotorTxHandler.removeFromMotorManager(*this); }

Tmotor_AK809::Tmotor_AK809(
    aruwsrc::Drivers* drivers,
    aruwsrc::motor::TMotorId desMotorIdentifier,
    tap::can::CanBus motorCanBus,
    bool isInverted,
    const char* name,
    uint16_t encoderWrapped,
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
      encoderWrapped(encoderWrapped),
      encoderRevolutions(encoderRevolutions)
{
    motorDisconnectTimeout.stop();
}

void Tmotor_AK809::initialize()
{
    drivers->tMotorTxHandler.addMotorToManager(this);
    attachSelfToRxHandler();
}

void Tmotor_AK809::processMessage(const modm::can::Message& message)
{
    if (message.getIdentifier() != Tmotor_AK809::getMotorIdentifier())
    {
        return;
    }
    uint16_t encoderActual =
        static_cast<uint16_t>(message.data[0] << 8 | message.data[1]);        // encoder value
    shaftRPM = static_cast<int16_t>(message.data[2] << 8 | message.data[3]);  // rpm
    shaftRPM = motorInverted ? -shaftRPM : shaftRPM;
    torque = static_cast<int16_t>(message.data[4] << 8 | message.data[5]);  // torque
    torque = motorInverted ? -torque : torque;
    temperature = static_cast<int8_t>(message.data[6]);  // temperature

    // restart disconnect timer, since you just received a message from the motor
    motorDisconnectTimeout.restart(MOTOR_DISCONNECT_TIME);

    // invert motor if necessary
    encoderActual = motorInverted ? ENC_RESOLUTION - 1 - encoderActual : encoderActual;
    updateEncoderValue(encoderActual);
}

void Tmotor_AK809::setDesiredOutput(int32_t desiredOutput)
{
    int32_t desOutputNotInverted = tap::algorithms::limitVal<int32_t>(desiredOutput, -60000, 60000);
    this->desiredOutput = motorInverted ? -desOutputNotInverted : desOutputNotInverted;
}

bool Tmotor_AK809::isMotorOnline() const
{
    /*
     * motor online if the disconnect timout has not expired (if it received message but
     * somehow got disconnected) and the timeout hasn't been stopped (initially, the timeout
     * is stopped)
     */
    return !motorDisconnectTimeout.isExpired() && !motorDisconnectTimeout.isStopped();
}

void Tmotor_AK809::serializeCanSendData(modm::can::Message* txMessage) const
{
    txMessage->data[0] = this->getOutputDesired() >> 24;
    txMessage->data[1] = this->getOutputDesired() >> 16;
    txMessage->data[2] = this->getOutputDesired() >> 8;
    txMessage->data[3] = this->getOutputDesired();
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

int64_t Tmotor_AK809::getEncoderUnwrapped() const
{
    return static_cast<int64_t>(encoderWrapped) +
           static_cast<int64_t>(ENC_RESOLUTION) * encoderRevolutions;
}

uint16_t Tmotor_AK809::getEncoderWrapped() const { return encoderWrapped; }

void Tmotor_AK809::updateEncoderValue(uint16_t newEncWrapped)
{
    int16_t enc_dif = newEncWrapped - encoderWrapped;
    if (enc_dif < -ENC_RESOLUTION / 2)
    {
        encoderRevolutions++;
    }
    else if (enc_dif > ENC_RESOLUTION / 2)
    {
        encoderRevolutions--;
    }
    encoderWrapped = newEncWrapped;
}
}  // namespace motor

}  // namespace aruwsrc
