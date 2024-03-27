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

#ifndef MULTI_SHOT_CV_COMMAND_MAPPING_HPP_
#define MULTI_SHOT_CV_COMMAND_MAPPING_HPP_

#include <optional>

#include "tap/control/hold_repeat_command_mapping.hpp"

#include "aruwsrc/control/agitator/constant_velocity_agitator_command.hpp"
#include "aruwsrc/control/governor/cv_on_target_governor.hpp"

#include "manual_fire_rate_reselection_manager.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::agitator
{
/**
 * Class that stores and allows the user to set some LaunchMode. Possible launch modes include
 * single, 10 Hz, or 20 Hz full auto mode.
 *
 * This object is a HoldRepeatCommandMapping. An instance of this object should be added to the
 * global CommandMapper to use it. This object contains a launch Command that it will schedule. How
 * often/when the launch Command will be schedule is based on the launch mode. This object controls
 * the fire rate and firing frequency of the launch Command based on the launch mode.
 *
 * If vision is running, the fire rate should not be limited and the launcher should be in full auto
 * mode, so this object checks the launchMode of a CvOnTargetGovernor when setting the fire rate.
 */
class MultiShotCvCommandMapping : public tap::control::HoldRepeatCommandMapping
{
public:
    /**
     * State of the shooting mechanism, how many times the associated command mapping should
     * reschedule the command when the mapping is met.
     */
    enum LaunchMode : uint8_t
    {
        SINGLE = 0,
        NO_HEATING,
        FULL_AUTO_10HZ,
        FULL_AUTO,
        NUM_SHOOTER_STATES,
    };

    /**
     * @param[in] drivers Reference to global drivers object.
     * @param[in] launchCommand Command that when scheduled launches a single projectile.
     * @param[in] rms The remote mapping that controls when the launch command should be scheduled.
     * @param[in] fireRateReselectionManager An optional argument, the fire rate reselection manager
     * that controls the fire rate of the launch command. If provided, the manager's fire rate is
     * updated based on the current LaunchMode.
     * @param[in] cvOnTargetGovernor The governor whose state will be used to override the current
     * LaunchMode. This allows us to override user-defined launch mode when CV is controlling the
     * launching frequency.
     */
    MultiShotCvCommandMapping(
        tap::Drivers &drivers,
        tap::control::Command &launchCommand,
        const tap::control::RemoteMapState &rms,
        std::optional<ManualFireRateReselectionManager *> fireRateReselectionManager,
        governor::CvOnTargetGovernor &cvOnTargetGovernor,
        ConstantVelocityAgitatorCommand &command);

    void setShooterState(LaunchMode mode)
    {
        if (mode < NUM_SHOOTER_STATES)
        {
            this->launchMode = mode;
        }
    }

    LaunchMode getLaunchMode() const { return launchMode; }

    void executeCommandMapping(const tap::control::RemoteMapState &currState);

private:
    std::optional<ManualFireRateReselectionManager *> fireRateReselectionManager;
    governor::CvOnTargetGovernor &cvOnTargetGovernor;

    LaunchMode launchMode = SINGLE;
    ConstantVelocityAgitatorCommand &command;

    int getCurrentBarrelCoolingRate() const
    {
        if (drivers->refSerial.getRobotData().turret.heatCoolingRate17ID1 != 0)
        {
            return drivers->refSerial.getRobotData().turret.heatCoolingRate17ID1 / 10.0f;
        }
        else if (drivers->refSerial.getRobotData().turret.heatCoolingRate17ID2 != 0)
        {
            return drivers->refSerial.getRobotData().turret.heatCoolingRate17ID2 / 10.0f;
        }
        else if (drivers->refSerial.getRobotData().turret.heatCoolingRate42 != 0)
        {
            return drivers->refSerial.getRobotData().turret.heatCoolingRate42 / 100.0f;
        }
        else
        {
            return ManualFireRateReselectionManager::MAX_FIRERATE_RPS;
        }
    }
};

}  // namespace aruwsrc::control::agitator

#endif  // MULTI_SHOT_CV_COMMAND_MAPPING_HPP_
