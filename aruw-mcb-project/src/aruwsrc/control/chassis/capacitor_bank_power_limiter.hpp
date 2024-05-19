/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ARUWSRC_POWER_LIMITER_HPP_
#define ARUWSRC_POWER_LIMITER_HPP_

#include "tap/communication/gpio/analog.hpp"
#include "tap/communication/sensors/current/current_sensor_interface.hpp"
#include "tap/control/chassis/power_limiter.hpp"

#include "aruwsrc/communication/can/capacitor_bank.hpp"

namespace tap
{
class Drivers;
}

namespace aruwsrc::chassis
{

class CapacitorSelectingCurrentSensor : public tap::communication::sensors::current::CurrentSensorInterface
{
public:
    CapacitorSelectingCurrentSensor(
        tap::communication::sensors::current::CurrentSensorInterface *currentSensor,
        aruwsrc::communication::can::capbank::CapacitorBank *capacitorBank);

    float getCurrentMa() const override;

    void update() override { this->currentSensor->update(); };
private:
    tap::communication::sensors::current::CurrentSensorInterface *currentSensor;
    aruwsrc::communication::can::capbank::CapacitorBank *capacitorBank;
};

class CapBankPowerLimiter : tap::control::chassis::PowerLimiter
{
public:
    CapBankPowerLimiter(
        const tap::Drivers* drivers,
        tap::communication::sensors::current::CurrentSensorInterface* currentSensor,
        aruwsrc::communication::can::capbank::CapacitorBank* capacitorBank,
        float startingEnergyBuffer,
        float energyBufferLimitThreshold,
        float energyBufferCritThreshold);

    float getPowerLimitRatio();

private:
    const tap::Drivers* drivers;
    const aruwsrc::communication::can::capbank::CapacitorBank* capacitorBank;
    CapacitorSelectingCurrentSensor sensor;

    const float LOWEST_CAP_VOLTAGE = 10.0f;
    const float POWER_RAMPDOWN_RANGE = 5.0f;
};
}  // namespace aruwsrc::chassis

#endif  // ARUWSRC_POWER_LIMITER_HPP_