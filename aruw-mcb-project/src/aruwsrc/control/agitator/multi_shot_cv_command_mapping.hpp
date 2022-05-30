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

#ifndef MULTI_SHOT_HANDLER_HPP_
#define MULTI_SHOT_HANDLER_HPP_

#include <optional>

#include "tap/control/hold_repeat_command_mapping.hpp"

#include "aruwsrc/control/governor/cv_on_target_governor.hpp"

#include "manual_fire_rate_reselection_manager.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::agitator
{
/**
 * Class that stores and allows the user to set some ShooterState. Possible shooter states include
 * single, 10 Hz, or 20 Hz full auto mode.
 *
 * This object is a HoldRepeatCommandMapping. It will update its `maxTimesToSchedule` based on the
 * ShooterState. Furthermore, this object modifies the fire rate of a
 * ManualFireRateReselectionManager.
 *
 * If vision is running, the fire rate should not be limited and the launcher should be in full auto
 * mode, so this object checks the state of a CvOnTargetGovernor before setting the fire rate.
 */
class MultiShotCvCommandMapping : public tap::control::HoldRepeatCommandMapping
{
public:
    /**
     * State of the shooting mechanism, how many times the associated command mapping should
     * reschedule the command when the mapping is met.
     */
    enum ShooterState : uint8_t
    {
        SINGLE = 0,
        FULL_AUTO_10HZ,
        FULL_AUTO,
        NUM_SHOOTER_STATES,
    };

    /**
     * @param[in] commandMapping The HoldRepeatCommandMapping whose `maxTimesToSchedule` variable to
     * update.
     */
    MultiShotCvCommandMapping(
        aruwsrc::Drivers &drivers,
        tap::control::Command &launchCommand,
        const tap::control::RemoteMapState &rms,
        std::optional<ManualFireRateReselectionManager *> fireRateManager,
        governor::CvOnTargetGovernor &cvOnTargetGovernor);

    void setShooterState(ShooterState state) { this->state = state; }

    ShooterState getShooterState() const { return state; }

    void executeCommandMapping(const tap::control::RemoteMapState &currState);

private:
    std::optional<ManualFireRateReselectionManager *> fireRateManager;
    governor::CvOnTargetGovernor &cvOnTargetGovernor;

    ShooterState state = SINGLE;
};

}  // namespace aruwsrc::control::agitator

#endif  // MULTI_SHOT_HANDLER_HPP_
