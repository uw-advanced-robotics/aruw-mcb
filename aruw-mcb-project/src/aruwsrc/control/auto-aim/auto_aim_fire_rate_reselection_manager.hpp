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

#ifndef AUTO_AIM_FIRE_RATE_RESELECTION_MANAGER_HPP_
#define AUTO_AIM_FIRE_RATE_RESELECTION_MANAGER_HPP_

#include "tap/errors/create_errors.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/agitator/fire_rate_reselection_manager_interface.hpp"
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
class AutoAimFireRateReselectionManager
    : public control::agitator::FireRateReselectionManagerInterface
{
public:
    static constexpr float LOW_RPS = 3;
    static constexpr float MID_RPS = 10;
    static constexpr float HIGH_RPS = 20;

    /**
     * @param[in] drivers Pointer to global drivers object.
     * @param[in] turretCVCommand
     * @param[in] turretID ID of the turret that this governor controls
     */
    AutoAimFireRateReselectionManager(
        aruwsrc::Drivers &drivers,
        const aruwsrc::control::turret::cv::TurretCVCommandInterface &turretCVCommand,
        const uint8_t turretID)
        : drivers(drivers),
          turretCVCommand(turretCVCommand),
          turretID(turretID)
    {
    }

    inline uint32_t getFireRatePeriod() final
    {
        auto fireRate = drivers.visionCoprocessor.getLastAimData(turretID).pva.firerate;
        switch (fireRate)
        {
            case aruwsrc::serial::VisionCoprocessor::FireRate::ZERO:
                return 0;
            case aruwsrc::serial::VisionCoprocessor::FireRate::LOW:
                return rpsToPeriodMS(LOW_RPS);
            case aruwsrc::serial::VisionCoprocessor::FireRate::MEDIUM:
                return rpsToPeriodMS(MID_RPS);
            case aruwsrc::serial::VisionCoprocessor::FireRate::HIGH:
                return rpsToPeriodMS(HIGH_RPS);
            default:
                RAISE_ERROR((&drivers), "Illegal fire rate value encountered");
                return 0;
        }
    }

    inline control::agitator::FireRateReadinessState getFireRateReadinessState() final
    {
        if (!drivers.commandScheduler.isCommandScheduled(&turretCVCommand))
        {
            // Don't limit firing if in manual fire mode
            return control::agitator::FireRateReadinessState::READY_IGNORE_RATE_LIMITING;
        }

        if (!drivers.visionCoprocessor.isCvOnline())
        {
            // We're in CV mode; prevent firing altogether if CV offline
            return control::agitator::FireRateReadinessState::NOT_READY;
        }

        if (drivers.visionCoprocessor.getLastAimData(turretID).pva.firerate ==
            aruwsrc::serial::VisionCoprocessor::FireRate::ZERO)
        {
            return control::agitator::FireRateReadinessState::NOT_READY;
        }

        return control::agitator::FireRateReadinessState::READY_USE_RATE_LIMITING;
    }

private:
    aruwsrc::Drivers &drivers;
    const aruwsrc::control::turret::cv::TurretCVCommandInterface &turretCVCommand;
    const uint8_t turretID;
};
}  // namespace aruwsrc::control::auto_aim

#endif  // AUTO_AIM_FIRE_RATE_RESELECTION_MANAGER_HPP_
