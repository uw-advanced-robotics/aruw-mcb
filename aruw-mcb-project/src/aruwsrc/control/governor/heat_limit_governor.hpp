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

#ifndef HEAT_LIMIT_GOVERNOR_HPP_
#define HEAT_LIMIT_GOVERNOR_HPP_

#include <cassert>

#include "tap/architecture/clock.hpp"
#include "tap/control/governor/command_governor_interface.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/ref_system_constants.hpp"

namespace aruwsrc::control::governor
{
/**
 * Governor that blocks Commands from running if the referee-reported heat limit is too high. Use to
 * avoid running commands that cause ref-system overheating.
 */
template <uint32_t HISTORY_WINDOW_MS = 200>
class HeatLimitGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    /**
     * @param firingSystemMechanismID ID of the barrel used to determine heat
     * @param heatLimitBuffer Amount of extra heat on top of cost of next projectile to wait for
     * before allowing firing
     */
    HeatLimitGovernor(
        tap::Drivers &drivers,
        const tap::communication::serial::RefSerialData::Rx::MechanismID firingSystemMechanismID,
        const uint16_t heatLimitBuffer,
        float heatRateDerivativeTrigger = 130.0f,
        float ballHeatCostMultiplier = 3.0f)
        : drivers(drivers),
          firingSystemMechanismID(firingSystemMechanismID),
          heatLimitBuffer(heatLimitBuffer),
          heatRateDerivativeTrigger(heatRateDerivativeTrigger),
          ballHeatCostMultiplier(ballHeatCostMultiplier)
    {
        static_assert(
            HISTORY_WINDOW_MS >= SAMPLE_INTERVAL_MS,
            "History window must be larger than sample interval.");
    }

    bool isReady() final { return enoughHeatToLaunchProjectile(); }

    bool isFinished() final { return !enoughHeatToLaunchProjectile(); }

private:
    tap::Drivers &drivers;
    const tap::communication::serial::RefSerialData::Rx::MechanismID firingSystemMechanismID;
    const uint16_t heatLimitBuffer;
    const float heatRateDerivativeTrigger;
    const float ballHeatCostMultiplier;

    static constexpr uint32_t SAMPLE_INTERVAL_MS = 10;  // Minimum time between recording samples

    // Buffer size based on window and interval.
    static constexpr size_t BUFFER_SIZE = HISTORY_WINDOW_MS / SAMPLE_INTERVAL_MS;

    struct HeatSample
    {
        uint32_t timeMs;
        int32_t heat;  // Signed to easily handle heat drops
    };

    HeatSample heatHistory[BUFFER_SIZE] = {};
    size_t historyHead = 0;
    size_t historyCount = 0;
    uint32_t lastSampleTimeMs = 0;

    float heatRate = 0.0f;  // Current calculated derivative (heat per second)

    bool enoughHeatToLaunchProjectile()
    {
        if (!drivers.refSerial.getRefSerialReceivingData())
        {
            return true;
        }

        const auto &robotData = drivers.refSerial.getRobotData();

        uint16_t heatLimit = robotData.turret.heatLimit;
        uint16_t nextCost = 0;
        uint16_t currentHeat = 0;

        switch (firingSystemMechanismID)
        {
            case tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM:
                currentHeat = robotData.turret.heat17;
                nextCost = aruwsrc::constants::HEAT_COST_17MM;
                break;
            case tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_42MM:
                currentHeat = robotData.turret.heat42;
                nextCost = aruwsrc::constants::HEAT_COST_42MM;
                break;
            default:
                // don't perform heat limiting
                currentHeat = 0;
                nextCost = 0;
                heatLimit = heatLimitBuffer;
        }

        updateHeatDerivative(currentHeat);

        /// @todo: remove this hardcode/make this system better thought out
        if (firingSystemMechanismID ==
            tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM)
        {
            if (heatRate > heatRateDerivativeTrigger)
            {
                nextCost = static_cast<uint16_t>(nextCost * ballHeatCostMultiplier);
            }
        }

        const bool heatBelowLimit = currentHeat + nextCost + heatLimitBuffer <= heatLimit;

        return !tap::communication::serial::RefSerial::heatAndLimitValid(currentHeat, heatLimit) ||
               heatBelowLimit;
    }

    /**
     * @brief Records a sample into the circular buffer if enough time has passed,
     * and calculates the heat rate over the entire recorded window.
     */
    void updateHeatDerivative(uint16_t currentHeat)
    {
        uint32_t currentTimeMs = tap::arch::clock::getTimeMilliseconds();

        // Sample rate limiter: only record data if at least SAMPLE_INTERVAL_MS has passed
        // as to not have a huge buffer
        if (currentTimeMs - lastSampleTimeMs >= SAMPLE_INTERVAL_MS)
        {
            heatHistory[historyHead] = {currentTimeMs, currentHeat};
            historyHead = (historyHead + 1) % BUFFER_SIZE;

            if (historyCount < BUFFER_SIZE)
            {
                historyCount++;
            }
            lastSampleTimeMs = currentTimeMs;
        }

        // Calculate rate based on the oldest and newest sample in the window
        if (historyCount > 1)
        {
            // If buffer isn't full, oldest is at index 0. If full, oldest is at historyHead.
            size_t oldestIdx = (historyCount < BUFFER_SIZE) ? 0 : historyHead;
            // Newest is the index just before historyHead
            size_t newestIdx = (historyHead == 0) ? BUFFER_SIZE - 1 : historyHead - 1;

            const auto &oldest = heatHistory[oldestIdx];
            const auto &newest = heatHistory[newestIdx];

            float dt = static_cast<float>(newest.timeMs - oldest.timeMs) / 1000.0f;

            if (dt > 0.0f)
            {
                heatRate = static_cast<float>(newest.heat - oldest.heat) / dt;
            }
        }
    }
};
}  // namespace aruwsrc::control::governor

#endif  // HEAT_LIMIT_GOVERNOR_HPP_