/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef MULTI_SHOT_CV_COMMAND_HPP_
#define MULTI_SHOT_CV_COMMAND_HPP_

#include <optional>

#include "tap/control/command.hpp"
#include "tap/control/finite_repeat_command.hpp"
#include "tap/control/repeat_command.hpp"

#include "aruwsrc/control/agitator/constant_velocity_agitator_command.hpp"
#include "aruwsrc/control/governor/cv_on_target_governor.hpp"

#include "manual_fire_rate_reselection_manager.hpp"

namespace aruwsrc::control::agitator
{
class MultiShotCvCommand : public tap::control::Command
{
public:
    enum LaunchMode : uint8_t
    {
        SINGLE = 0,
        NO_HEATING,
        LIMITED_10HZ,
        LIMITED_20HZ,
        FULL_AUTO,
        NUM_SHOOTER_STATES,
    };

    MultiShotCvCommand(
        tap::Drivers& drivers,
        tap::control::Command& launchCommand,
        std::optional<ManualFireRateReselectionManager*> fireRateReselectionManager,
        governor::CvOnTargetGovernor& cvOnTargetGovernor,
        std::optional<ConstantVelocityAgitatorCommand*> command = std::nullopt);

    void setShooterState(LaunchMode mode)
    {
        if (mode < NUM_SHOOTER_STATES)
        {
            launchMode = mode;
        }
    }

    LaunchMode getLaunchMode() const { return launchMode; }

    bool isReady() override;

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "MultiShotCvCommand"; }

private:
    tap::Drivers& drivers;
    tap::control::Command& launchCommand;
    // for continuous shot shoot modes
    tap::control::RepeatCommand repeatCommand;
    tap::control::Command* activeCommand = nullptr;
    std::optional<ManualFireRateReselectionManager*> fireRateReselectionManager;
    governor::CvOnTargetGovernor& cvOnTargetGovernor;
    std::optional<ConstantVelocityAgitatorCommand*> command;
    bool singleShotFinished = false;
    bool activeCommandEnded = false;
#if defined(ALL_STANDARDS)
    LaunchMode launchMode = LIMITED_20HZ;
#else
    LaunchMode launchMode = SINGLE;
#endif

    bool initializedActiveCommand = false;

    int getCurrentBarrelCoolingRate() const
    {
        int coolingRate = drivers.refSerial.getRobotData().turret.coolingRate;

#if defined(TARGET_HERO_PERSEUS)
        return coolingRate / 100.0f;
#else
        return coolingRate / 10.0f;
#endif
    }
};

}  // namespace aruwsrc::control::agitator

#endif