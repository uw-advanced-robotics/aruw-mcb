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

#ifndef AS5600_REGISTER_HPP_
#define AS5600_REGISTER_HPP_

namespace aruwsrc::communication::sensors::as5600v2
{

#define AS5600_ADDRESS 0x36
#define AS5600_READ_BIT 0x8

// Configuration registers
#define AS5600_REG_ZMCO 0x00       // How many times the BURN command has been executed
#define AS5600_REG_ZPOS_HIGH 0x01  // Zero position high-4 bits
#define AS5600_REG_ZPOS_LOW 0x02   // Zero position low-8 bits
#define AS5600_REG_MPOS_HIGH 0x03  // Max position high-4 bits
#define AS5600_REG_MPOS_LOW 0x04   // Max position low-8 bits
#define AS5600_REG_MANG_HIGH 0x05  // Maximum angle high-4 bits
#define AS5600_REG_MANG_LOW 0x06   // Maximum angle low-8 bits
#define AS5600_REG_CONF_HIGH 0x07  // Configuration high-4 bits
#define AS5600_REG_CONF_LOW 0x08   // Configuration low-8 bits

// Output registers
#define AS5600_REG_RAW_ANGLE_HIGH 0x0C  // Raw angle high-8 bits
#define AS5600_REG_RAW_ANGLE_LOW 0x0D   // Raw angle low-8 bits
#define AS5600_REG_ANGLE_HIGH 0x0E      // Angle high-8 bits
#define AS5600_REG_ANGLE_LOW 0x0F       // Angle low-8 bits

// Status registers
#define AS5600_REG_STATUS 0x0B          // Status register
#define AS5600_REG_AGC 0x1A             // Automatic gain control value
#define AS5600_REG_MAGNITUDE_HIGH 0x1B  // Magnitude of the magnetic field high-8 bits
#define AS5600_REG_MAGNITUDE_LOW 0x1C   // Magnitude of the magnetic field low-8 bits

// Burn commands
#define AS5600_REG_BURN 0xFF  // Burn command

// -------- Values for configs --------
#define AS5600_POWER_MODE_NORMAL 0x00
#define AS5600_POWER_MODE_LOW_POWER_MODE_1 0x01
#define AS5600_POWER_MODE_LOW_POWER_MODE_2 0x02
#define AS5600_POWER_MODE_LOW_POWER_MODE_3 0x03

#define AS5600_HYSTERESIS_OFF 0x00
#define AS5600_HYSTERESIS_1LSB 0x01
#define AS5600_HYSTERESIS_2LSB 0x02
#define AS5600_HYSTERESIS_3LSB 0x03

#define AS5600_OUTPUT_STAGE_ANALOG_FULL 0x00
#define AS5600_OUTPUT_STAGE_ANALOG_REDUCED_TO_90 0x01
#define AS5600_OUTPUT_STAGE_PWM 0x02

#define AS5600_PWM_FREQUENCY_115HZ 0x00
#define AS5600_PWM_FREQUENCY_230HZ 0x01
#define AS5600_PWM_FREQUENCY_460HZ 0x02
#define AS5600_PWM_FREQUENCY_920HZ 0x03

#define AS5600_SLOW_FILTER_16X 0x00
#define AS5600_SLOW_FILTER_8X 0x01
#define AS5600_SLOW_FILTER_4X 0x02
#define AS5600_SLOW_FILTER_2X 0x03

#define AS5600_FAST_FILTER_OFF 0x00
#define AS5600_FAST_FILTER_6LSB 0x01
#define AS5600_FAST_FILTER_7LSB 0x02
#define AS5600_FAST_FILTER_9LSB 0x03
#define AS5600_FAST_FILTER_18LSB 0x04
#define AS5600_FAST_FILTER_21LSB 0x05
#define AS5600_FAST_FILTER_24LSB 0x06
#define AS5600_FAST_FILTER_10LSB 0x07

#define AS5600_WATCHDOG_OFF 0x00
#define AS5600_WATCHDOG_ON 0x01

// -------- Configurations --------
#define AS5600_ZPOS_HIGH_CONFIG 0x00  // Start at 0
#define AS5600_ZPOS_LOW_CONFIG 0x00
#define AS5600_MPOS_HIGH_CONFIG 0x0F  // Go till max which should be 360
#define AS5600_MPOS_LOW_CONFIG 0xFF
#define AS5600_MANG_HIGH_CONFIG 0x0F
#define AS5600_MANG_LOW_CONFIG 0xFF

#define AS5600_CONF_HIGH_CONFIG \
    ((AS5600_WATCHDOG_OFF << 5) | (AS5600_FAST_FILTER_OFF << 2) | AS5600_SLOW_FILTER_16X)
#define AS5600_CONF_LOW_CONFIG                                                    \
    ((AS5600_PWM_FREQUENCY_115HZ << 6) | (AS5600_OUTPUT_STAGE_ANALOG_FULL << 4) | \
     (AS5600_HYSTERESIS_OFF << 2) | AS5600_POWER_MODE_NORMAL)


}  // namespace aruwsrc::communication::sensors::as5600v2

#endif
