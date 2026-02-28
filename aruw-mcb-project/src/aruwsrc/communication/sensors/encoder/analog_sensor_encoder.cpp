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

#include "aruwsrc/communication/sensors/encoder/analog_sensor_encoder.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"

#include "modm/math/geometry/angle.hpp"

namespace aruwsrc::communication::sensors::encoder
{
namespace
{
uint32_t computeResolution(const AnalogSensorEncoder::Calibration& calibration)
{
    if (calibration.rawMax <= calibration.rawMin)
    {
        return 1;
    }

    return static_cast<uint32_t>(calibration.rawMax - calibration.rawMin + 1);
}

uint32_t computeHomePosition(const AnalogSensorEncoder::Calibration& calibration)
{
    uint16_t clampedZero =
        tap::algorithms::limitVal(calibration.rawZero, calibration.rawMin, calibration.rawMax);
    return static_cast<uint32_t>(clampedZero - calibration.rawMin);
}
}  // namespace

AnalogSensorEncoder::AnalogSensorEncoder(
    aruwsrc::communication::can::AruwAnalogSensor* sensor,
    Channel channel,
    const Calibration& calibration,
    bool isInverted)
    : WrappedEncoder(
          isInverted,
          computeResolution(calibration),
          calibration.outputRangeRadians / static_cast<float>(M_TWOPI),
          computeHomePosition(calibration)),
      sensor(sensor),
      channel(channel),
      calibration(calibration),
      lastUpdateMicros(0)
{
    if (this->sensor != nullptr)
    {
        this->sensor->addUpdateListener(this);
    }
}

AnalogSensorEncoder::~AnalogSensorEncoder()
{
    if (this->sensor != nullptr)
    {
        this->sensor->removeUpdateListener(this);
    }
}

bool AnalogSensorEncoder::isOnline() const
{
    return (this->sensor != nullptr) && this->sensor->isOnline();
}

tap::algorithms::WrappedFloat AnalogSensorEncoder::getPosition() const
{
    const_cast<AnalogSensorEncoder*>(this)->updateFromSensor();
    return WrappedEncoder::getPosition();
}

float AnalogSensorEncoder::getVelocity() const
{
    const_cast<AnalogSensorEncoder*>(this)->updateFromSensor();
    return WrappedEncoder::getVelocity();
}

void AnalogSensorEncoder::update() { updateFromSensor(); }

void AnalogSensorEncoder::onAnalogSensorUpdated() { updateFromSensor(); }

void AnalogSensorEncoder::logTelemetry(
    communication::rtt::RttTelemetry& rttTelemetry,
    const char* namePrefix) const
{
    if (this->sensor == nullptr)
    {
        return;
    }

    this->sensor->sendRTTTelemetry(&rttTelemetry, namePrefix);
}

void AnalogSensorEncoder::updateFromSensor()
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

    uint16_t raw = readRaw();
    updateEncoderValue(rawToTicks(raw));
}

uint16_t AnalogSensorEncoder::readRaw() const
{
    if (this->channel == Channel::AI0)
    {
        return this->sensor->getAnalogInput0();
    }

    return this->sensor->getAnalogInput1();
}

uint32_t AnalogSensorEncoder::rawToTicks(uint16_t raw) const
{
    uint16_t clamped = tap::algorithms::limitVal(raw, calibration.rawMin, calibration.rawMax);

    return static_cast<uint32_t>(clamped - calibration.rawMin);
}

}  // namespace aruwsrc::communication::sensors::encoder
