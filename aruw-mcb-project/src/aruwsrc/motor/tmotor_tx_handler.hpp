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

#ifndef TMOTOR_TX_HANDLER_HPP_
#define TMOTOR_TX_HANDLER_HPP_

#include <limits.h>

#include "tap/util_macros.hpp"

#include "tmotor_ak80-9.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::motor
{
/**
 * Converts the dji MotorId to a uint32_t.
 * @param[in] id Some CAN MotorId
 * @return id normalized to be around [0, DJI_MOTORS_PER_CAN), or some value >= DJI_MOTORS_PER_CAN
 * if the id is out of bounds
 */
#define TMOTOR_TO_NORMALIZED_ID(id)                                                    \
    static_cast<uint32_t>(                                                                \
        (id < aruwsrc::motor::MOTOR1) ? (aruwsrc::motor::TMotorTxHandler::TMOTOR_MOTORS_PER_CAN) \
                                  : (id - aruwsrc::motor::MOTOR1))

/**
 * Converts the dji MotorId to a uint32_t.
 * @param[in] idx Some index, a motor id index normalized between [0, DJI_MOTORS_PER_CAN)
 * @return idx, converted to a MotorId
 */
#define NORMALIZED_ID_TO_TMOTOR(idx)   \
    static_cast<aruwsrc::motor::MotorId>( \
        idx + static_cast<int32_t>(aruwsrc::motor::MotorId::MOTOR1))

/**
 * Uses modm can interface to send CAN packets to tmotor ak80-9's connected to the two CAN buses.
 *
 * To use this class properly, declare a motor somewhere, then call the initialize method, which
 * allows one to start interacting with a motor connected via CAN bus. When the motor's `initialize`
 * function is called, this object's `addMotorToManager` function is called and the motor is ready
 * to have its control information sent to the motor on the bus.
 *
 * To send messages, call this class's `encodeAndSendCanData` function.
 */
class TMotorTxHandler
{
public:
    /** Number of motors on each CAN bus. */
    static constexpr int TMOTOR_MOTORS_PER_CAN = 8;
    /** CAN message length of each motor control message. */
    static constexpr int CAN_TMOTOR_MESSAGE_SEND_LENGTH = 8;

    TMotorTxHandler(aruwsrc::Drivers* drivers) : drivers(drivers) {}
    mockable ~TMotorTxHandler() = default;
    DISALLOW_COPY_AND_ASSIGN(TMotorTxHandler)

    /**
     * Adds the motor to the manager so that it can receive motor messages from the CAN bus. If
     * there is already a motor with the same ID in the manager, the program will abort
     */
    mockable void addMotorToManager(Tmotor_AK809* motor);

    /**
     * Sends motor commands across the CAN bus. Sends up to 4 messages (2 per CAN bus), though it
     * may send less depending on which motors have been registered with the motor manager. Each
     * messages encodes motor controller command information for up to 4 motors.
     */
    mockable void encodeAndSendCanData();

    /**
     * Removes the motor from the motor manager.
     */
    mockable void removeFromMotorManager(const Tmotor_AK809& motor);

    mockable Tmotor_AK809 const* getCan1Motor(TMotorId motorId);

    mockable Tmotor_AK809 const* getCan2Motor(TMotorId motorId);

private:
    aruwsrc::Drivers* drivers;

    Tmotor_AK809* can1MotorStore[TMOTOR_MOTORS_PER_CAN] = {0};
    Tmotor_AK809* can2MotorStore[TMOTOR_MOTORS_PER_CAN] = {0};

    void addMotorToManager(Tmotor_AK809** canMotorStore, Tmotor_AK809* const motor);

    void serializeMotorStoreSendData(
        Tmotor_AK809* motor,
        modm::can::Message* message,
        bool* validMotorMessage);

    void removeFromMotorManager(const Tmotor_AK809& motor, Tmotor_AK809** motorStore);

};

}  // namespace aruwsrc::motor

#endif  // TMOTOR_TX_HANDLER_HPP_
