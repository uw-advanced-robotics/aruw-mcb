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

#include "tap/algorithms/math_user_utils.hpp"

#include "modm/architecture/interface/register.hpp"
#include "modm/math/utils.hpp"

#ifndef ISM330_DATA_HPP_
#define ISM330_DATA_HPP_
namespace aruwsrc::communication::sensors::imu
{
enum Register : uint8_t
{
    DEVICE_ADDRESS = 0x6B,
    WHO_AM_I = 0x0F,

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

enum XL_Config : uint8_t
{
    G2_CONFIG = 0b11110000,
    G16_CONFIG = 0b11110100,
    G4_CONFIG = 0b11111000,
    G8_CONFIG = 0b11111100
};

enum Gyro_Config : uint8_t
{  // Also capable of 125 and 4000 dps
    DPS250_CONFIG = 0b11110000,
    DPS500_CONFIG = 0b11110100,
    DPS1000_CONFIG = 0b11111000,
    DPS2000_CONFIG = 0b11111100
};

enum ODR : uint8_t
{  // Only includes high power
    ODR_416HZ = 0b01101111,
    ODR_833HZ = 0b01111111,
    ODR_1660HZ = 0b10001111,
    ODR_3330HZ = 0b10011111,
    ODR_6660HZ = 0b10101111
};

#define READ_LENGTH 14

static constexpr float TEMPERATURE_OFFSET = 25.0f;
static constexpr float TEMPERATURE_SENSITIVITY = 256.0f;

}  // namespace aruwsrc::communication::sensors::imu

#endif  // ISM330_DATA_HPP_
