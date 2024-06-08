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

#ifndef CAPACITOR_BANK_POWER_LIMITER_HPP_
#define CAPACITOR_BANK_POWER_LIMITER_HPP_

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
static constexpr float VOLTAGE_RAMPDOWN_RANGE = 5.0f;

static constexpr float K_I = 0.002;
static constexpr float K_P = 0.005;

class CapacitorSelectingCurrentSensor
    : public tap::communication::sensors::current::CurrentSensorInterface
{
public:
    CapacitorSelectingCurrentSensor(
        tap::communication::sensors::current::CurrentSensorInterface* currentSensor,
        can::capbank::CapacitorBank* capacitorBank);

    float getCurrentMa() const override;

    void update() override { this->currentSensor->update(); };

private:
    tap::communication::sensors::current::CurrentSensorInterface* currentSensor;
    can::capbank::CapacitorBank* capacitorBank;
};

class CapBankPowerLimiter
{
public:
    CapBankPowerLimiter(
        const tap::Drivers* drivers,
        tap::communication::sensors::current::CurrentSensorInterface* currentSensor,
        can::capbank::CapacitorBank* capacitorBank,
        float startingEnergyBuffer,
        float energyBufferLimitThreshold,
        float energyBufferCritThreshold);

    float getPowerLimitRatio();

private:
    const tap::Drivers* drivers;
    const can::capbank::CapacitorBank* capacitorBank;
    CapacitorSelectingCurrentSensor sensor;

    tap::control::chassis::PowerLimiter fallback;

    float currentIntegrator = 0;
};
}  // namespace aruwsrc::chassis

#endif  // CAPACITOR_BANK_POWER_LIMITER_HPP_