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

#include "aruwsrc/communication/can/aruw_pressure_sensor.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"

#include "modm/math/geometry/angle.hpp"

namespace aruwsrc::communication::can
{
AruwPressureSensor::AruwPressureSensor(
    aruwsrc::communication::can::AruwAnalogSensor* sensor,
    Channel channel,
    const Calibration& calibration)
    : pressureScalar(
          float(calibration.pressureMax - calibration.pressureMin) /
          float(calibration.rawMax - calibration.rawMin)),
      sensor(sensor),
      channel(channel),
      calibration(calibration),
      lastUpdateMicros(0)
{
}

bool AruwPressureSensor::isOnline() const
{
    return (this->sensor != nullptr) && this->sensor->isOnline();
}

float AruwPressureSensor::getPressurekPascals() const { return rawTokPascals(updateFromSensor()); }

void AruwPressureSensor::update() { updateFromSensor(); }

void AruwPressureSensor::logTelemetry(
    communication::rtt::RttTelemetry& rttTelemetry,
    const char* namePrefix) const
{
    if (this->sensor == nullptr)
    {
        return;
    }

    this->sensor->sendRTTTelemetry(&rttTelemetry, namePrefix);
}

uint16_t AruwPressureSensor::updateFromSensor() const
{
    if (this->sensor == nullptr)
    {
        return 0;
    }

    uint32_t now = tap::arch::clock::getTimeMicroseconds();
    if (now == lastUpdateMicros)
    {
        return raw;
    }
    lastUpdateMicros = now;

    raw = readRaw();
    return raw;
}

uint16_t AruwPressureSensor::readRaw() const
{
    if (this->channel == Channel::AI0)
    {
        return this->sensor->getAnalogInput0();
    }

    return this->sensor->getAnalogInput1();
}

float AruwPressureSensor::rawTokPascals(uint16_t raw) const
{
    uint16_t clamped = tap::algorithms::limitVal(raw, calibration.rawMin, calibration.rawMax);

    return static_cast<float>(clamped - calibration.rawMin) * pressureScalar +
           calibration.pressureMin;
}

}  // namespace aruwsrc::communication::can
