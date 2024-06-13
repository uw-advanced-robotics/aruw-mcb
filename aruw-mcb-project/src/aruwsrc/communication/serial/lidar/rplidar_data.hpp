/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef RPLIDAR_DATA_H_
#define RPLIDAR_DATA_H_

#include <cstdint>

namespace aruwsrc::communication::serial::lidar
{

static constexpr uint8_t START_BYTE = 0xA5;
// Only applicable for messages sent from LIDAR
static constexpr uint8_t START_BYTE_2 = 0x5A;

enum Commands : uint8_t
{
    STOP = 0x25,
    RESET = 0x40,
    SCAN = 0x20,
    EXPRESS_SCAN = 0x82,
    FORCE_SCAN = 0x21,
    GET_INFO = 0x50,
    GET_HEALTH = 0x52,
    GET_SAMPLERATE = 0x59,
    GET_LIDAR_CONF = 0x84
};



struct RPLidarResponse
{
    uint8_t startFlag1;
    uint8_t startFlag2;
    // 2 bits reserved for send mode
    uint32_t dataSizeandMode;
    uint8_t dataType;
};

/**
 * Protocol for measurement as such:
 * 
 * Quality:
 * 0 bit is whether the data is part of a new scan
 * 1 bit is always inverse of first bit, data checker
 * Bits [8:2] Signal strength of laser pulse
 * 
 * Angle:
 * 0 bit is always 1
 * Remaining bits is the angle, actual angle is value divided by 64.0 degrees
 * 
 * Distance:
 * Distance as measured in millimeters, set to 0 when invalid.
 * Actual distance computed when dividing value by 4.0mm
 * 
 */
struct RPLidarMeasurement
{
    uint8_t quality; 
    uint16_t angle;
    uint16_t distance;
};

}; // namespace aruwsrc::communication::serial::lidar

#endif // RPLIDAR_DATA_H_
