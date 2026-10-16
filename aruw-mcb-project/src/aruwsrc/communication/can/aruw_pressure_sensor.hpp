/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ARUW_PRESSURE_SENSOR_HPP_
#define ARUW_PRESSURE_SENSOR_HPP_

#include "tap/communication/sensors/encoder/wrapped_encoder.hpp"
#include "tap/util_macros.hpp"

#include "aruwsrc/communication/can/aruw_analog_sensor.hpp"
#include "aruwsrc/communication/rtt/rtt_telemetry.hpp"

namespace aruwsrc::communication::can
{
class AruwPressureSensor
{
public:
    struct Calibration
    {
        uint16_t rawMin;
        uint16_t rawMax;
        int16_t pressureMin;
        int16_t pressureMax;
    };

    enum class Channel
    {
        AI0,
        AI1
    };

    AruwPressureSensor(
        aruwsrc::communication::can::AruwAnalogSensor* sensor,
        Channel channel,
        const Calibration& calibration);

    void initialize() {}

    bool isOnline() const;

    float getPressurekPascals() const;

    void update();

    void logTelemetry(
        communication::rtt::RttTelemetry& rttTelemetry,
        const char* namePrefix = "sensor:pressure_sensor") const;

    DISALLOW_COPY_AND_ASSIGN(AruwPressureSensor)

private:
    uint16_t updateFromSensor() const;
    uint16_t readRaw() const;
    mutable uint16_t raw = 0;
    const float pressureScalar;
    float rawTokPascals(uint16_t raw) const;

    aruwsrc::communication::can::AruwAnalogSensor* sensor;
    Channel channel;
    const Calibration& calibration;
    mutable uint32_t lastUpdateMicros;
};

}  // namespace aruwsrc::communication::can

#endif  // ARUW_PRESSURE_SENSOR_HPP_
