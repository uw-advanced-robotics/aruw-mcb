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

#ifndef LAMPREY_ENCODER_HPP_
#define LAMPREY_ENCODER_HPP_

#include "modm/container/pair.hpp"
#include "modm/math/interpolation/linear.hpp"

#include "analog_sensor_encoder.hpp"

namespace aruwsrc::communication::sensors::encoder
{
class LampreyEncoder : public AnalogSensorEncoder
{
public:
    using Calibration = AnalogSensorEncoder::Calibration;

    /***
     * @param sensor the AruwAnalogSensor that this encoder will read from
     * @param channel the channel of the AruwAnalogSensor that this encoder will read from
     * @param calibration the calibration values for this encoder
     * @param lookupTableConfig a lookup table mapping raw encoder ticks to positions in radians.
     *     The raw encoder ticks should be in the range of [0, calibration.rawMax -
     * calibration.rawMin]. The position values should be in the range of [0,
     * calibration.outputRangeRadians].
     * @param isInverted whether the positive rotation direction of the shaft is clockwise or
     * counter clockwise.
     */
    template <std::size_t LUT_SIZE>
    LampreyEncoder(
        aruwsrc::communication::can::AruwAnalogSensor* sensor,
        AnalogSensorEncoder::Channel channel,
        const Calibration& calibration,
        const modm::Pair<float, float> (&lookupTableConfig)[LUT_SIZE],
        bool isInverted = false)
        : AnalogSensorEncoder(sensor, channel, calibration, isInverted),
          lookupTable(lookupTableConfig, LUT_SIZE)
    {
    }

    uint16_t getRaw() const { return raw; }
    uint32_t getTicks() const { return ticks; }

protected:
    void updateFromSensor() override
    {
        if (this->sensor == nullptr)
        {
            return;
        }

        uint32_t now = tap::arch::clock::getTimeMicroseconds();
        if (now == lastUpdateMicros)
        {
            return;
        }
        lastUpdateMicros = now;

        const uint16_t rawClamped =
            std::clamp(this->readRaw(), calibration.rawMin, calibration.rawMax);

        // Get raw ticks
        ticks = rawToTicks(rawClamped);

        // Get the non-linear position in radians from your lookup table
        const float positionRadians = lookupTable.interpolate(static_cast<float>(ticks));

        // Calculate the encoder's resolution (matching the base class logic)
        uint32_t resolution = 1;
        if (calibration.rawMax > calibration.rawMin)
        {
            resolution = calibration.rawMax - calibration.rawMin + 1;
        }

        // Convert radians back to linearized ticks
        float fraction = positionRadians / calibration.outputRangeRadians;
        uint32_t linearizedTicks = static_cast<uint32_t>(fraction * static_cast<float>(resolution));

        updateEncoderValue(linearizedTicks);
    };

private:
    modm::interpolation::Linear<modm::Pair<float, float>> lookupTable;
    uint_fast16_t raw{0};
    uint32_t ticks{0};
};

}  // namespace aruwsrc::communication::sensors::encoder

#endif  // ARUW_ANALOG_SENSOR_ENCODER_HPP_
