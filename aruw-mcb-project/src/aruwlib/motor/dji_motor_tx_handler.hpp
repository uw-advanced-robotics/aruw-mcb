/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

/*
 * This file is part of ARUW's repository.
 *
 * Uses modm can interface to send CAN packets to dji motors. Receiving
 * data is done on the motor level in conjunction with
 * can_receive_interface.hpp.
 *
 * When a motor is instantiated, a CAN interface and a motor number is
 * requested. This class checks when a motor is created and kills
 * the board if multiple identical motors are created.
 *
 * To use this class properly, declare a motor somewhere, preferibly
 * in static memory, and then call the initialize method, which allows
 * one to start interacting with a motor connected via CAN bus. When
 * initialize is called, the motor class is added to a list of motors
 * which a handler runs through, sending and receiving messages for all
 * motors.
 *
 * To receive messages, create a new motor and call the initialize
 * function. This adds the motor to a queue of receive message handlers
 * that is updated by calling the pollCanData function.
 *
 * To send messages, call processCanSendMessage.
 *
 * Currently, no error handling is implemented if you attempt to use
 * a motor without initialization. This must be implemented when
 * error handling is complete.
 */

#ifndef __MOTOR_MANAGER_HPP__
#define __MOTOR_MANAGER_HPP__

#include <limits.h>

#include "aruwlib/communication/serial/TerminalSerial.hpp"

#include "dji_motor.hpp"
#include "mock_macros.hpp"

namespace aruwlib
{
class Drivers;
namespace motor
{
#define DJI_MOTOR_NORMALIZED_ID(id) ((int32_t)id - aruwlib::motor::MotorId::MOTOR1)

class DjiMotorTxHandler : public communication::serial::ITerminalSerialCallback
{
public:
    DjiMotorTxHandler(Drivers* drivers) : drivers(drivers) {}
    DjiMotorTxHandler(const DjiMotorTxHandler&) = delete;
    DjiMotorTxHandler& operator=(const DjiMotorTxHandler&) = delete;
    mockable ~DjiMotorTxHandler() = default;

    // Called when a motor is created, adds to the motor manager
    // and checks to make sure the motor is not already being used.
    mockable void addMotorToManager(DjiMotor* motor);

    mockable void processCanSendData();

    mockable void removeFromMotorManager(const DjiMotor& motor);

    mockable DjiMotor const* getCan1MotorData(MotorId motorId);

    mockable DjiMotor const* getCan2MotorData(MotorId motorId);

    std::string terminalSerialCallback(std::stringstream&& inputLine) override;

private:
    static const int DJI_MOTORS_PER_CAN = 8;

    Drivers* drivers;

    DjiMotor* can1MotorStore[DJI_MOTORS_PER_CAN] = {0};
    DjiMotor* can2MotorStore[DJI_MOTORS_PER_CAN] = {0};

    void addMotorToManager(DjiMotor** canMotorStore, DjiMotor* const motor);

    void serializeMotorStoreSendData(
        DjiMotor** canMotorStore,
        modm::can::Message* messageLow,
        modm::can::Message* messageHigh);

    void removeFromMotorManager(const DjiMotor& motor, DjiMotor** motorStore);

    void zeroTxMessage(modm::can::Message* message);

    std::string getMotorInfoToString(const DjiMotor& motor);

    void printAllMotorInfo(DjiMotor** motorStore, std::string& outStr);
};

}  // namespace motor

}  // namespace aruwlib

#endif
