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
CapBankPowerLimiter::CapBankPowerLimiter(
    const tap::Drivers *drivers,
    tap::communication::sensors::current::CurrentSensorInterface *currentSensor,
    aruwsrc::communication::sensors::power::ExternalCapacitorBank *capacitorBank,
    float startingEnergyBuffer,
    float energyBufferLimitThreshold,
    float energyBufferCritThreshold)
    : fallbackLimiter(
          drivers,
          currentSensor,
          startingEnergyBuffer,
          energyBufferLimitThreshold,
          energyBufferCritThreshold),
      drivers(drivers),
      capacitorBank(capacitorBank)
{
}

float CapBankPowerLimiter::getPowerLimitRatio(float desiredCurrent)
{
    if (drivers->refSerial.getRefSerialReceivingData() &&
        (drivers->refSerial.getRobotData().currentHp == 0 ||
         (drivers->refSerial.getRobotData().robotPower.value & 0b010) == 0))
    {
        return 0;
    }

    float fallbackLimit = fallbackLimiter.getPowerLimitRatio();

    if (this->capacitorBank == nullptr ||
        this->capacitorBank->getStatus() !=
            aruwsrc::communication::sensors::power::Status::CHARGE_DISCHARGE)
    {
        return fallbackLimit;
    }

    // current limit
    float currentLimit = desiredCurrent != 0
                             ? tap::algorithms::limitVal(15'000.0f / desiredCurrent, 0.0f, 1.0f)
                             : 1.0;

    // volage limit
    float capVoltageLimit = tap::algorithms::limitVal(
        (this->capacitorBank->getVoltage() - LOWEST_CAP_VOLTAGE) / POWER_RAMPDOWN_RANGE,
        0.0f,
        1.0f);

        return this->capacitorBank->getSprintModifer() *
           std::min(std::min(fallbackLimit, currentLimit), capVoltageLimit);
}

}  // namespace aruwsrc::chassis
