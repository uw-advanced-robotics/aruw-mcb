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

#ifndef ANALOG_SENSOR_ENCODER_HPP_
#define ANALOG_SENSOR_ENCODER_HPP_

#include "tap/communication/sensors/encoder/wrapped_encoder.hpp"
#include "tap/util_macros.hpp"

#include "aruwsrc/communication/can/aruw_analog_sensor.hpp"
#include "aruwsrc/communication/rtt/rtt_telemetry.hpp"

namespace aruwsrc::communication::sensors::encoder
{
class AnalogSensorEncoder : public tap::encoder::WrappedEncoder
{
public:
    enum class Channel
    {
        AI0,
        AI1
    };

    struct Calibration
    {
        uint16_t rawMin;
        uint16_t rawMax;
        uint16_t rawZero;
        float outputRangeRadians;
    };

    AnalogSensorEncoder(
        aruwsrc::communication::can::AruwAnalogSensor* sensor,
        Channel channel,
        const Calibration& calibration,
        bool isInverted = false);

    void initialize() override {}

    bool isOnline() const override;

    tap::algorithms::WrappedFloat getPosition() const override;
    float getVelocity() const override;

    void update();

    void logTelemetry(
        communication::rtt::RttTelemetry& rttTelemetry,
        const char* namePrefix = "ANALOG_SENSOR") const;

    DISALLOW_COPY_AND_ASSIGN(AnalogSensorEncoder)

private:
    void updateFromSensor();
    uint16_t readRaw() const;
    uint32_t rawToTicks(uint16_t raw) const;

    aruwsrc::communication::can::AruwAnalogSensor* sensor;
    Channel channel;
    Calibration calibration;
    mutable uint32_t lastUpdateMicros;
};

}  // namespace aruwsrc::communication::sensors::encoder

#endif  // ARUW_ANALOG_SENSOR_ENCODER_HPP_
