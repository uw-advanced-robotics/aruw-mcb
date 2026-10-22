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

#include "tap/architecture/timeout.hpp"
#include "tap/communication/can/can.hpp"
#include "tap/communication/sensors/encoder/can_encoder/can_encoder.hpp"

#include "modm/container/pair.hpp"
#include "modm/math/interpolation/linear.hpp"

namespace aruwsrc::communication::sensors::encoder
{
class LampreyEncoder : public tap::encoder::CanEncoder
{
public:
    /**
     * @brief 0 length array version
     */
    LampreyEncoder(
        tap::Drivers* drivers,
        tap::encoder::CanEncoderId CAN_ID,
        tap::can::CanBus CAN_BUS,
        const modm::Pair<float, float> (&)[0],
        bool isInverted = false)
        : CanEncoder(drivers, CAN_ID, CAN_BUS, isInverted),
          lookupTable(nullptr, 0),
          lutSize(0)
    {
    }
    /**
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
        tap::Drivers* drivers,
        tap::encoder::CanEncoderId CAN_ID,
        tap::can::CanBus CAN_BUS,
        const modm::Pair<float, float> (&lookupTableConfig)[LUT_SIZE],
        bool isInverted = false)
        : CanEncoder(drivers, CAN_ID, CAN_BUS, isInverted),
          lookupTable(lookupTableConfig, LUT_SIZE),
          lutSize(LUT_SIZE),
          powerOnTimeout()
    {
    }

    float getRawAngle() const { return angleRaw; }

    bool isOnline() const override
    {
        return powerOnTimeout.isExpired() && CanEncoder::CanEncoder::isOnline();
    }

    void initialize() override
    {
        CanEncoder::CanEncoder::initialize();
        powerOnTimeout.restart(500);
    }

    void processMessage(const modm::can::Message& message)
    {
        uint16_t raw = (message.data[1] << 8) | message.data[0];

        angleRaw = modm::toRadian(raw / 100.0f);

        float angle = angleRaw;

        if (lutSize > 0)
        {
            angle = lookupTable.interpolate(angleRaw);
        }

        if (inverted)
        {
            angle = -angle;
        }

        angle -= encoderHomePosition.getWrappedValue();
        if (angle < 0.0f)
        {
            angle += M_TWOPI;
        }

        if (lastUpdateTime == 0)
        {
            encoder = tap::algorithms::WrappedFloat(angle, 0, M_TWOPI);
        }
        else
        {
            encoder += encoder.minDifference(angle);
        }

        uint32_t time = tap::arch::clock::getTimeMicroseconds();
        deltaTime = time - this->lastUpdateTime;
        this->lastUpdateTime = time;

        pastPosition = position;
        position.setUnwrappedValue(encoder.getUnwrappedValue());

        this->gauss = 0.0f;
        this->encoderDisconnectTimeout.restart(DISCONNECT_TIME);
    }

private:
    modm::interpolation::Linear<modm::Pair<float, float>> lookupTable;

    const size_t lutSize{0};
    float angleRaw{0.0f};

    tap::arch::MilliTimeout powerOnTimeout;
};

}  // namespace aruwsrc::communication::sensors::encoder

#endif  // ARUW_ANALOG_SENSOR_ENCODER_HPP_
