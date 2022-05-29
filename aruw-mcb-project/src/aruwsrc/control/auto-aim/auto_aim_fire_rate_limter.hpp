/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef AUTO_AIM_FIRE_RATE_LIMITER_HPP_
#define AUTO_AIM_FIRE_RATE_LIMITER_HPP_

#include "tap/errors/create_errors.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/turret/cv/turret_cv_command_interface.hpp"
#include "aruwsrc/drivers.hpp"

namespace aruwsrc::control::auto_aim
{
/**
 * Limits the frequency with which the underlying command is scheduled to be at most the last
 * "fire rate" suggestion provided by the Vision Coprocessor for this turret.
 *
 * If CV is disconnected, does not limit fire.
 */
class AutoAimFireRateLimiter
{
public:
    static constexpr float LOW_RPS = 3;
    static constexpr float MID_RPS = 10;
    static constexpr float HIGH_RPS = 20;

    /**
     * @param[in] drivers Pointer to global drivers object.
     * @param[in] visionCoprocessor Vision coprocessor communicator.
     * @param[in] turretID ID of the turret that this governor controls
     */
    AutoAimFireRateLimiter(
        aruwsrc::Drivers &drivers,
        const aruwsrc::serial::VisionCoprocessor &visionCoprocessor,
        const aruwsrc::control::turret::cv::TurretCVCommandInterface &turretCVCommand,
        const uint8_t turretID)
        : drivers(drivers),
          visionCoprocessor(visionCoprocessor),
          turretCVCommand(turretCVCommand),
          turretID(turretID)
    {
    }

    inline uint32_t getFireRatePeriod(aruwsrc::serial::VisionCoprocessor::FireRate fireRate)
    {
        float fireRateRps = 0;

        auto fireRate = visionCoprocessor.getLastAimData(turretID).firerate;
        switch (fireRate)
        {
            case aruwsrc::serial::VisionCoprocessor::FireRate::ZERO:
                return UINT32_MAX;
            case aruwsrc::serial::VisionCoprocessor::FireRate::LOW:
                fireRateRps = LOW_RPS;
                break;
            case aruwsrc::serial::VisionCoprocessor::FireRate::MEDIUM:
                fireRateRps = MID_RPS;
                break;
            case aruwsrc::serial::VisionCoprocessor::FireRate::HIGH:
                fireRateRps = HIGH_RPS;
                break;
            default:
                RAISE_ERROR((&drivers), "Illegal fire rate value encountered");
                fireRateRps = 0;
                break;
        }

        return rpsToPeriodMS(fireRateRps);
    }

    inline bool fireRateReady()
    {
        if (!drivers.commandScheduler.isCommandScheduled(&turretCVCommand))
        {
            // Don't limit firing if in manual fire mode
            return true;
        }

        if (!visionCoprocessor.isCvOnline())
        {
            // We're in CV mode; prevent firing altogether if CV offline
            return false;
        }

        if (visionCoprocessor.getLastAimData(turretID).firerate ==
            aruwsrc::serial::VisionCoprocessor::FireRate::ZERO)
        {
            return false;
        }

        return true;
    }

private:
    aruwsrc::Drivers &drivers;
    const aruwsrc::serial::VisionCoprocessor &visionCoprocessor;
    const aruwsrc::control::turret::cv::TurretCVCommandInterface &turretCVCommand;
    const uint8_t turretID;

    /**
     * Converts a rounds-per-second value (i.e., Hz) to a period in milliseconds.
     */
    static inline constexpr uint32_t rpsToPeriodMS(float rps) { return (1000.0f / rps); }
};
}  // namespace aruwsrc::control::auto_aim

#endif  // AUTO_AIM_FIRE_RATE_LIMITER_HPP_
