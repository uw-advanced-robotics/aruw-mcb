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

#ifndef FIRE_RATE_LIMIT_GOVERNOR_HPP_
#define FIRE_RATE_LIMIT_GOVERNOR_HPP_

#include "tap/architecture/periodic_timer.hpp"
#include "tap/control/governor/command_governor_interface.hpp"
#include "tap/errors/create_errors.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/turret/cv/turret_cv_command_interface.hpp"
#include "aruwsrc/drivers.hpp"
#include "modm/processing/timer/periodic_timer.hpp"

namespace aruwsrc::control::governor
{
/**
 * A governor that allows a Command to run based on an internal timer and information from the
 * vision processor that dictates fire rate.
 *
 * Limits the frequency with which the underlying command is scheduled to be at most the last
 * "fire rate" suggestion provided by the Vision Coprocessor for this turret.
 *
 * If CV is disconnected, does not limit fire.
 */
class FireRateLimitGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    /**
     * @param[in] drivers Pointer to global drivers object.
     * @param[in] visionCoprocessor Vision coprocessor communicator.
     * @param[in] turretID ID of the turret that this governor controls
     */
    FireRateLimitGovernor(
        aruwsrc::Drivers &drivers,
        aruwsrc::serial::VisionCoprocessor &visionCoprocessor,
        aruwsrc::control::turret::cv::TurretCVCommandInterface &turretCVCommand,
        uint8_t turretID)
        : drivers(drivers),
          visionCoprocessor(visionCoprocessor),
          turretCVCommand(turretCVCommand),
          turretID(turretID)
    {
    }

    void initialize() final { restartTimer(); }

    bool isReady() final
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

        if (timer.isStopped())
        {
            restartTimer();
        }

        if (visionCoprocessor.getLastAimData(turretID).firerate ==
            aruwsrc::serial::VisionCoprocessor::FireRate::ZERO)
        {
            return false;
        }

        return timer.isExpired();
    }

    bool isFinished() final
    {
        // Never explicitly force the command to stop. We assume that it will fire at most one shot
        // then end by itself.
        return false;
    }

private:
    aruwsrc::Drivers &drivers;
    aruwsrc::serial::VisionCoprocessor &visionCoprocessor;
    aruwsrc::control::turret::cv::TurretCVCommandInterface &turretCVCommand;
    uint8_t turretID;

    tap::arch::MilliTimeout timer;
    static constexpr float LOW_RPS = 3;
    static constexpr float MID_RPS = 10;
    static constexpr float HIGH_RPS = 20;

    void restartTimer()
    {
        switch (visionCoprocessor.getLastAimData(turretID).firerate)
        {
            case aruwsrc::serial::VisionCoprocessor::FireRate::ZERO:  // don't fire
                break;
            case aruwsrc::serial::VisionCoprocessor::FireRate::LOW:  // low fire rate
                timer.restart(rpsToPeriodMS(LOW_RPS));
                break;
            case aruwsrc::serial::VisionCoprocessor::FireRate::MEDIUM:  // medium fire rate
                timer.restart(rpsToPeriodMS(MID_RPS));
                break;
            case aruwsrc::serial::VisionCoprocessor::FireRate::HIGH:  // high fire rate
                timer.restart(rpsToPeriodMS(HIGH_RPS));
                break;
            default:
                RAISE_ERROR((&drivers), "Illegal fire rate value encountered");
                break;
        }
    }

    static inline constexpr uint32_t rpsToPeriodMS(float rps) { return (1000.0f / rps); }
};
}  // namespace aruwsrc::control::governor

#endif  // FIRE_RATE_LIMIT_GOVERNOR_HPP_
