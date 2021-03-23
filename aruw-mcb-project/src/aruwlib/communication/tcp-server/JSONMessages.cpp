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

#ifdef PLATFORM_HOSTED

#include "JSONMessages.hpp"

#include <iostream>
#include <string>

#include "aruwlib/motor/dji_motor.hpp"

namespace aruwlib
{
namespace communication
{
/**
 * The JSON namespace provides methods for working with JSON strings
 * which the MCB-simulator will be sending to the Windows simulator
 * and potentially vice versa.
 */
namespace json
{
using std::cout;
using std::string;

static constexpr char MESSAGE_TERMINATOR[] = "\n\r\n\r";

/**
 * returns JSON string representing motor message.
 */
string makeMotorMessage(const aruwlib::motor::DjiMotor& motor)
{
    string jsonMessage =
        "{\"messageType\":\"motor\","
        "\"canBus\":" +
        std::to_string(static_cast<int>(motor.getCanBus()) + 1) + "," +
        "\"motorID\":" + std::to_string(motor.getMotorIdentifier()) + "," +
        "\"shaftRPM\":" + std::to_string(motor.getShaftRPM()) + "," +
        "\"torque\":" + std::to_string(motor.getTorque()) + "," +
        "\"encoderValue\":" + std::to_string(motor.getEncoderUnwrapped()) + "}" +
        MESSAGE_TERMINATOR;
    return jsonMessage;
}

}  // namespace json

}  // namespace communication

}  // namespace aruwlib

#endif  // PLATFORM_HOSTED
