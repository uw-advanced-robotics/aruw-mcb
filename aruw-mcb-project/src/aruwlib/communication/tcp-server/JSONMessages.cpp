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

#ifdef PLATFORM_HOSTED

#include "JSONMessages.hpp"
#include "aruwlib/motor/dji_motor.hpp"
#include <iostream>
#include <string>

namespace aruwlib 
{
namespace communication 
{
/**
 * The JSON namespace provides methods for working with JSON strings
 * which the MCB-simulator will be sending to the Windows simulator
 * and potentially vice versa.
 */
namespace JSON 
{
using std::string;
using std::cout;

/**
 * returns JSON string representing motor message. (Null-terminated)
 */
string makeMotorMessage(const aruwlib::motor::DjiMotor& motor) {
    string jsonMessage = 
        "{\"messageType\": \"motor\", "
        "\"motorID\": " + std::to_string(motor.getMotorIdentifier()) + ", " +
        "\"shaftRPM\": " + std::to_string(motor.getShaftRPM()) + ", " +
        "\"torque\": " + std::to_string(motor.getTorque()) + ", " +
        "\"encoderValue\": " + 
            std::to_string(motor.encStore.getEncoderUnwrapped()) +
        "}\0";
    return jsonMessage;
}

const char* makeMotorMessageLiteral(const aruwlib::motor::DjiMotor& motor) {
    return makeMotorMessage(motor).c_str();
}

} // namespace JSON

} // namespace communication

} // namespace aruwlib

#endif // PLATFORM_HOSTED
