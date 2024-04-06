/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef VIRTUAL_CURRENT_SENSOR_HPP_
#define VIRTUAL_CURRENT_SENSOR_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/sensors/current/current_sensor_interface.hpp"

#include "virtual_analog.hpp"

namespace aruwsrc::virtualMCB
{
class VirtualCurrentSensor : public tap::communication::sensors::current::CurrentSensorInterface
{
    /**
     * Parameters for the analog current sensor.
     */
    struct Config
    {
        const VirtualAnalog *analogDriver;
        const tap::gpio::Analog::Pin analogSensorPin;
        /**
         * The conversion factor from millivolts (read in by the analog driver) to current in
         * milliamps
         *
         * The sensor returns an analog reading in mV. `currentSensorMaPerMv` has units mA/mV, so
         * multiplying the analog reading by `currentSensorMaPerMv` results in a value in mV.
         */
        const float currentSensorMaPerMv;
        /**
         * When zero milliamps are being read, the raw analog value that the sensor reports in
         * millivolts
         */
        const float currentSensorZeroMv;
        /**
         * Alpha gain to be used to low pass filter the raw analog data.
         */
        const float currentSensorLowPassAlpha;
    };

public:
    VirtualCurrentSensor(const Config &config) : config(config) {}

    float getCurrentMa() const { return current; }

    void update()
    {
        current = tap::algorithms::lowPassFilter(
            current,
            abs(static_cast<float>(config.analogDriver->read(config.analogSensorPin)) -
                config.currentSensorZeroMv) *
                config.currentSensorMaPerMv,
            config.currentSensorLowPassAlpha);
    }

private:
    const Config config;

    float current = 0;
};
}  // namespace aruwsrc::virtualMCB

#endif  // VIRTUAL_CURRENT_SENSOR_HPP_
