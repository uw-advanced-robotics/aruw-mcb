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

#ifndef ISM330_DATA_HPP_
#define ISM330_DATA_HPP_

#include "tap/algorithms/math_user_utils.hpp"

#include "modm/architecture/interface/register.hpp"
#include "modm/math/utils.hpp"

namespace aruwsrc::communication::sensors::imu::ism330
{
enum Register : uint8_t
{
    DEVICE_ADDRESS = 0x6A,
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

enum AccelerometerRangeConfig : uint8_t
{
    G2_CONFIG = (0b00 << 2) | 0x0,
    G16_CONFIG = (0b01 << 2) | 0x0,
    G4_CONFIG = (0b10 << 2) | 0x0,
    G8_CONFIG = (0b11 << 2) | 0x0,
    G_CONFIG_BITMASK = 0b11110011
};

enum GyroscopeRangeConfig : uint8_t
{  // Also capable of 125 and 4000 dps
    DPS250_CONFIG = (0b00 << 2) | 0x0,
    DPS500_CONFIG = (0b01 << 2) | 0x0,
    DPS1000_CONFIG = (0b10 << 2) | 0x0,
    DPS2000_CONFIG = (0b11 << 2) | 0x0,
    DPS_CONFIG_BITMASK = 0b11110011
};

enum OutputDataRate : uint8_t
{  // Only includes high power
    ODR_416HZ = (0b0110 << 4) | 0x0,
    ODR_833HZ = (0b0111 << 4) | 0x0,
    ODR_1660HZ = (0b1000 << 4) | 0x0,
    ODR_3330HZ = (0b1001 << 4) | 0x0,
    ODR_6660HZ = (0b1010 << 4) | 0x0,
    ODR_BITMASK = 0b00001111
};

#define READ_LENGTH 14

static constexpr float TEMPERATURE_OFFSET = 25.0f;
static constexpr float TEMPERATURE_SENSITIVITY = 256.0f;

}  // namespace aruwsrc::communication::sensors::imu::ism330

#endif  // ISM330_DATA_HPP_
