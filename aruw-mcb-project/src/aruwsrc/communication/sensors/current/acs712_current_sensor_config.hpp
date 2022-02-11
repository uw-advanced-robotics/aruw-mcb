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

#ifndef ACS712_CURRENT_SENSOR_CONFIG_HPP_
#define ACS712_CURRENT_SENSOR_CONFIG_HPP_

/**
 * Defines some reasonable current sensor constants for the ACS712 current sensor.
 *
 * See here for information on the ACS712 current sensor:
 * https://www.seeedstudio.com/blog/2020/02/15/acs712-current-sensor-features-how-it-works-arduino-guide/
 */
namespace aruwsrc::communication::sensors::current
{
/**
 * Calibrated current sensor's mv to ma ratio used to convert analog value to an actual
 * current. The current sensor we are using is linear, so only the slope and y-intercept
 * are needed.
 */
static constexpr float ACS712_CURRENT_SENSOR_MV_PER_MA = 11.47f;
/**
 * Voltage (in mV) that the current sensor reads 0 mA.
 */
static constexpr float ACS712_CURRENT_SENSOR_ZERO_MA = 3088.7f;

/**
 * A low pass filter is used to remove spikes, this is the alpha parameter that defines the low
 * pass filter.
 */
static constexpr float ACS712_CURRENT_SENSOR_LOW_PASS_ALPHA = 0.1f;
}  // namespace aruwsrc::communication::sensors::current

#endif  // ACS712_CURRENT_SENSOR_CONFIG_HPP_
