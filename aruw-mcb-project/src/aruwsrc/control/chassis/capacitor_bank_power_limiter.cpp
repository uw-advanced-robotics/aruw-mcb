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

float error = 0.0f;
float currentIntegratorCopy = 0.0f;

float CapBankPowerLimiter::getPowerLimitRatio()
{
    if (drivers->refSerial.getRefSerialReceivingData() &&
        (drivers->refSerial.getRobotData().currentHp == 0 ||
         (drivers->refSerial.getRobotData().robotPower.value & 0b010) == 0))
    {
        return 0;
    }

    float fallback = tap::control::chassis::PowerLimiter::getPowerLimitRatio();
    if (this->capacitorBank == nullptr ||
        this->capacitorBank->isDisabled() ||
        this->capacitorBank->getState() == communication::can::capbank::State::UNKNOWN ||
        this->capacitorBank->getState() == communication::can::capbank::State::SAFE)
    {
        return fallback;
    }

    float setpoint = drivers->refSerial.getRobotData().chassis.powerConsumptionLimit / 24.0f;

    if (this->capacitorBank->getSprinting() == communication::can::capbank::SprintMode::SPRINT) {
        setpoint = 6; // TODO: get this based on a table or something
    }

    float measured = this->capacitorBank->getCurrent();
    
    error = setpoint - measured;

    const float K_I = 0.001;
    this->currentIntegrator += K_I * error;
    
    const float K_P = 0.005;

    currentIntegratorCopy = this->currentIntegrator;

    this->currentIntegrator = std::clamp(this->currentIntegrator, -100.0f, 0.4f);

    float controlFractionOutput = std::clamp(this->currentIntegrator + (error * K_P), 0.0f, 1.0f);

    const float LOW_V_RAMP_RANGE = 3.0f;
    const float MIN_CAP_V = 10.0f;

    float lowVoltageRamp = std::clamp((this->capacitorBank->getVoltage() - MIN_CAP_V) / LOW_V_RAMP_RANGE, 0.0f, 1.0f);

    return controlFractionOutput * lowVoltageRamp;
}

}  // namespace aruwsrc::chassis