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

#ifndef MULTI_SHOT_COMMAND_MAPPING_HPP_
#define MULTI_SHOT_COMMAND_MAPPING_HPP_

#include <cassert>

#include "tap/control/command.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::turret::cv
{
class TurretCVCommand;
}

namespace aruwsrc::agitator
{
/**
 * Class that stores and allows the user to set some ShooterState. Possible shooter states include
 * single, burst, or full auto mode.
 *
 * The MultiShotCommandMapping has an associated HoldRepeatCommandMapping. When the shooter state is
 * set, the HoldRepeatCommandMapping's `maxTiemsToSchedule` is updated. Thus, assuming the
 * HoldRepeatCommandMapping is associated with some command that launches a projectile, this class
 * will control the "state" of the shooter.
 */
class MultiShotCommandMapping : public tap::control::HoldRepeatCommandMapping
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
        FULL_AUTO_20HZ,
        NUM_SHOOTER_STATES,
    };

    MultiShotCommandMapping(
        aruwsrc::Drivers &drivers,
        tap::control::Command &singleLaunchCommand,
        tap::control::Command &fullAuto10HzCommand,
        tap::control::Command &fullAuto20HzCommand,
        const aruwsrc::control::turret::cv::TurretCVCommand &turretCvCommand,
        const tap::control::RemoteMapState &rms);

    void executeCommandMapping(const tap::control::RemoteMapState &currState) override;

    void setShooterState(ShooterState state);

    ShooterState getShooterState() const { return state; }

private:
    aruwsrc::Drivers &drivers;
    tap::control::Command &singleLaunchCommand;
    tap::control::Command &fullAuto10HzCommand;
    tap::control::Command &fullAuto20HzCommand;
    const aruwsrc::control::turret::cv::TurretCVCommand &turretCvCommand;

    ShooterState state = SINGLE;

    inline tap::control::Command &getLaunchCommand(ShooterState s)
    {
        switch (s)
        {
            case SINGLE:
                return singleLaunchCommand;
            case FULL_AUTO_10HZ:
                return fullAuto10HzCommand;
            case FULL_AUTO_20HZ:
                return fullAuto20HzCommand;
            default:
                assert(false);
        }
    }

    static inline int getTimesToReschedule(ShooterState s)
    {
        switch (s)
        {
            case SINGLE:
                return 1;
            case FULL_AUTO_10HZ:
                return -1;
            case FULL_AUTO_20HZ:
                return -1;
            default:
                return -1;
        }
    }
};

}  // namespace aruwsrc::agitator

#endif  // MULTI_SHOT_COMMAND_MAPPING_HPP_
