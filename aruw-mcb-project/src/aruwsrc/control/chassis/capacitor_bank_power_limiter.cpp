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

namespace aruwsrc::control::chassis
{
CapacitorSelectingSensor::CapacitorSelectingSensor(
    tap::communication::sensors::current::CurrentSensorInterface *currentSensor,
    tap::communication::sensors::voltage::VoltageSensorInterface *voltageSensor,
    communication::can::cap_bank::CapacitorBank *capacitorBank)
    : currentSensor(currentSensor),
      voltageSensor(voltageSensor),
      capacitorBank(capacitorBank){};

float CapacitorSelectingSensor::getCurrentMa() const
{
    // Always return the real battery bus current from 0x1C5 so the Taproot
    // PowerLimiter tracks the same signal the referee monitors (P_battery).
    // Previously this returned cap rail current when the cap bank was online,
    // which caused the limiter to track P_cap instead of P_battery — a
    // different quantity that fights the cap bank's own power regulation.
    return currentSensor->getCurrentMa();
}

float CapacitorSelectingSensor::getVoltageMv() const
{
    // Always return the real battery bus voltage from 0x1C5 (same reasoning
    // as getCurrentMa — the Taproot PowerLimiter must see P_battery).
    return this->voltageSensor->getVoltageMv();
}

CapBankPowerLimiter::CapBankPowerLimiter(
    const tap::Drivers *drivers,
    tap::communication::sensors::current::CurrentSensorInterface *currentSensor,
    tap::communication::sensors::voltage::VoltageSensorInterface *voltageSensor,
    aruwsrc::communication::can::cap_bank::CapacitorBank *capacitorBank,
    float startingEnergyBuffer,
    float energyBufferLimitThreshold,
    float energyBufferCritThreshold)
    : drivers(drivers),
      capacitorBank(capacitorBank),
      sensor(currentSensor, voltageSensor, capacitorBank),
      fallback(
          drivers,
          &sensor,
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

    // Delegate to the Taproot PowerLimiter, which monitors real battery draw
    // from the 0x1C5 sensor and tracks the referee's energy buffer (Z).
    //
    // The cap bank firmware handles power regulation internally — it keeps
    // P_battery at (ref - 5W) via its own inner PID. At steady state,
    // P_battery < P_limit, so the fallback returns 1.0 (no throttle).
    // During transients (sprint start, cap bank ramp-up) P_battery may
    // briefly exceed P_limit; the fallback sees the buffer drain and
    // throttles motors to prevent a referee penalty — exactly the safety
    // net we need, without fighting the cap bank's controller.
    //
    // The previous cap-current PI loop was removed because it tracked cap
    // rail current (P_cap) instead of battery draw (P_battery), causing it
    // to fight the cap bank's inner PID at steady state.
    return this->fallback.getPowerLimitRatio();
}

}  // namespace aruwsrc::control::chassis