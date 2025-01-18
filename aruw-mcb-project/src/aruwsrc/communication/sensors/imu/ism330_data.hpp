/*
 * Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "modm/architecture/interface/register.hpp"
#include "modm/math/utils.hpp"
#include "tap/algorithms/math_user_utils.hpp"

#ifndef ISM330_DATA_HPP_
#define ISM330_DATA_HPP_
namespace aruwsrc::communication::sensors::ism330
{
    enum Register : uint8_t {
    DEVICE_ADDRESS = 0x6B,

    // temperature
    OUT_TEMP_L = 0x20,
    OUT_TEMP_H = 0x21,
    
    // gyroscope
    CTRL2_G = 0x11,
    OUTX_L_G = 0x22,
    OUTX_H_G = 0x23,
    OUTY_L_G = 0x24,
    OUTY_H_G = 0x25,
    OUTZ_L_G = 0x26,
    OUTZ_H_G = 0x27,

    // accelerometer
    CTRL1_XL = 0x10,
    OUTX_L_XL = 0x28,
    OUTX_H_XL = 0x29,
    OUTY_L_XL = 0x2A,
    OUTY_H_XL = 0x2B,
    OUTZ_L_XL = 0x2C,
    OUTZ_H_XL = 0x2D
};

struct ImuData
{
    enum Axis
    {
        X = 0,
        Y = 1,
        Z = 2,
    };

    float gyroRaw[3] = {};
    float accRaw[3] = {};
    float temperature;
};


#define ACCELEROMETER_CONFIG ((0b0111 << 4) | 0x0)  // 833hz, 2g, pg 49


#define GYRO_CONFIG ((0b0111 << 4) | 0x0)  // 833hz

#define READ_LENGTH 13

}  // namespace aruwsrc::communication::sensors::ism330

#endif //ISM330_DATA_HPP_
