/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "tmotor_tx_handler.hpp"

#include <cassert>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "aruwsrc/drivers.hpp"
#include "modm/architecture/interface/assert.h"
#include "modm/architecture/interface/can_message.hpp"

namespace aruwsrc::motor
{
void TMotorTxHandler::addMotorToManager(Tmotor_AK809** canMotorStore, Tmotor_AK809* const motor)
{
    assert(motor != nullptr);
    uint32_t idIndex = TMOTOR_TO_NORMALIZED_ID(motor->getMotorIdentifier());
    bool motorOverloaded = canMotorStore[idIndex] != nullptr;
    bool motorOutOfBounds = idIndex >= DJI_MOTORS_PER_CAN;
    modm_assert(!motorOverloaded && !motorOutOfBounds, "TMotorTxHandler", "overloading");
    canMotorStore[idIndex] = motor;
}

void TMotorTxHandler::addMotorToManager(Tmotor_AK809* motor)
{
    // add new motor to either the can1 or can2 motor store
    // because we checked to see if the motor is overloaded, we will
    // never have to worry about overfilling the CanxMotorStore array
    if (motor->getCanBus() == tap::can::CanBus::CAN_BUS1)
    {
        addMotorToManager(can1MotorStore, motor);
    }
    else
    {
        addMotorToManager(can2MotorStore, motor);
    }
}

void TMotorTxHandler::encodeAndSendCanData()
{
    // set up new can messages to be sent via CAN bus 1 and 2
    modm::can::Message can1Message(
        CAN_TMOTOR_LOW_IDENTIFIER,
        CAN_TMOTOR_MESSAGE_SEND_LENGTH,
        0,
        true);
    modm::can::Message can2Message(
        CAN_TMOTOR_LOW_IDENTIFIER,
        CAN_TMOTOR_MESSAGE_SEND_LENGTH,
        0,
        true);
    bool can1ValidMotorMessage = false;
    bool can2ValidMotorMessage = false;

    serializeMotorStoreSendData(can1MotorStore, &can1Message, &can1ValidMotorMessage);

    serializeMotorStoreSendData(can2MotorStore, &can2Message, &can2ValidMotorMessage);

    bool messageSuccess = true;

    if (drivers->can.isReadyToSend(tap::can::CanBus::CAN_BUS1))
    {
        if (can1ValidMotorMessage)
        {
            messageSuccess &= drivers->can.sendMessage(tap::can::CanBus::CAN_BUS1, can1Message);
        }
    }
    if (drivers->can.isReadyToSend(tap::can::CanBus::CAN_BUS2))
    {
        if (can2ValidMotorMessage)
        {
            debugmessage = can2Message;
            messageSuccess &= drivers->can.sendMessage(tap::can::CanBus::CAN_BUS2, can2Message);
        }
    }

    if (!messageSuccess)
    {
        RAISE_ERROR(drivers, "sendMessage failure");
    }
}

void TMotorTxHandler::serializeMotorStoreSendData(
    Tmotor_AK809** canMotorStore,
    modm::can::Message* message,
    bool* validMotorMessage)
{
    for (int i = 0; i < DJI_MOTORS_PER_CAN; i++)
    {
        const Tmotor_AK809* const motor = canMotorStore[i];
        if (motor != nullptr)
        {
            message->setIdentifier((uint32_t)i |((uint32_t)0x01 << 8));  // sets the CAN ID as well as setting "CURRENT LOOP MODE"
            motor->serializeCanSendData(message);
            *validMotorMessage = true;
        }
    }
}

void TMotorTxHandler::removeFromMotorManager(const Tmotor_AK809& motor)
{
    if (motor.getCanBus() == tap::can::CanBus::CAN_BUS1)
    {
        removeFromMotorManager(motor, can1MotorStore);
    }
    else
    {
        removeFromMotorManager(motor, can2MotorStore);
    }
}

void TMotorTxHandler::removeFromMotorManager(const Tmotor_AK809& motor, Tmotor_AK809** motorStore)
{
    uint32_t id = TMOTOR_TO_NORMALIZED_ID(motor.getMotorIdentifier());
    if (id > TMOTOR_TO_NORMALIZED_ID(aruwsrc::motor::MOTOR8) || motorStore[id] == nullptr)
    {
        RAISE_ERROR(drivers, "invalid motor id");
        return;
    }
    motorStore[id] = nullptr;
}

Tmotor_AK809 const* TMotorTxHandler::getCan1Motor(TMotorId motorId)
{
    uint32_t index = TMOTOR_TO_NORMALIZED_ID(motorId);
    return index > TMOTOR_TO_NORMALIZED_ID(aruwsrc::motor::MOTOR8) ? nullptr
                                                                   : can1MotorStore[index];
}

Tmotor_AK809 const* TMotorTxHandler::getCan2Motor(TMotorId motorId)
{
    uint32_t index = TMOTOR_TO_NORMALIZED_ID(motorId);
    return index > TMOTOR_TO_NORMALIZED_ID(aruwsrc::motor::MOTOR8) ? nullptr
                                                                   : can2MotorStore[index];
}
}  // namespace aruwsrc::motor
