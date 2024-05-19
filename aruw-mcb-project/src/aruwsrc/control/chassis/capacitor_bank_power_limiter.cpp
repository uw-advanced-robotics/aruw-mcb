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

#include "capacitor_bank_power_limiter.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"

namespace aruwsrc::chassis
{

CapacitorSelectingCurrentSensor::CapacitorSelectingCurrentSensor(
    tap::communication::sensors::current::CurrentSensorInterface *currentSensor,
    aruwsrc::communication::can::capbank::CapacitorBank *capacitorBank):

    currentSensor(currentSensor),
    capacitorBank(capacitorBank) {

    };

float CapacitorSelectingCurrentSensor::getCurrentMa() const {
    if (this->capacitorBank == nullptr || this->capacitorBank->getState() == communication::can::capbank::State::UNKNOWN)
    {
        return currentSensor->getCurrentMa();
    }

    return this->capacitorBank->getCurrent() * 1000;
}

CapBankPowerLimiter::CapBankPowerLimiter(
    const tap::Drivers *drivers,
    tap::communication::sensors::current::CurrentSensorInterface *currentSensor,
    aruwsrc::communication::can::capbank::CapacitorBank *capacitorBank,
    float startingEnergyBuffer,
    float energyBufferLimitThreshold,
    float energyBufferCritThreshold)
    : sensor(currentSensor, capacitorBank),
      drivers(drivers),
      capacitorBank(capacitorBank),
      tap::control::chassis::PowerLimiter(
          drivers,
          &sensor,
          startingEnergyBuffer,
          energyBufferLimitThreshold,
          energyBufferCritThreshold)
{
}

float CapBankPowerLimiter::getPowerLimitRatio()
{
    if (drivers->refSerial.getRefSerialReceivingData() &&
        (drivers->refSerial.getRobotData().currentHp == 0 ||
         (drivers->refSerial.getRobotData().robotPower.value & 0b010) == 0))
    {
        return 0;
    }

    float fallbackLimit = tap::control::chassis::PowerLimiter::getPowerLimitRatio();

    if (this->capacitorBank == nullptr ||
        this->capacitorBank->isDisabled() ||
        this->capacitorBank->getState() == communication::can::capbank::State::SAFE)
    {
        return fallbackLimit;
    }

    // volage limiting, lerps to the fallback as the caps are close to empty
    float capVoltageLimit = tap::algorithms::limitVal(
        (this->capacitorBank->getVoltage() - LOWEST_CAP_VOLTAGE) / POWER_RAMPDOWN_RANGE,
        0.0f,
        1.0f);

    if (this->capacitorBank->getSprinting() == communication::can::capbank::SprintMode::REGULAR) {
        return fallbackLimit;
    }

    return ((1 - capVoltageLimit) * fallbackLimit + capVoltageLimit);
}

}  // namespace aruwsrc::chassis