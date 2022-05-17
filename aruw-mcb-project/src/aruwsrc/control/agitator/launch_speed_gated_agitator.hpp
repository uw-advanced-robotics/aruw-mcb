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

#ifndef LAUNCH_SPEED_GATED_AGITATOR_HPP_
#define LAUNCH_SPEED_GATED_AGITATOR_HPP_

#include "../launcher/friction_wheel_subsystem.hpp"
#include "aruwsrc/drivers.hpp"

#include "agitator_subsystem.hpp"

namespace aruwsrc::control::agitator
{
/**
 * CHANGE !!!!!!!!!!!!!!!!!!!!!1
 */
class LaunchSpeedGatedAgitator : public aruwsrc::agitator::AgitatorSubsystem
{
public:
    /**
     * @param[in] drivers A pointer to the `Drivers` struct.
     * @param[in] agitatorSubsystem
     * @param[in] frictionWheelSubsystem
     * @param[in] launchSpeedThreshold
     *
     */
    LaunchSpeedGatedAgitator(
        aruwsrc::Drivers* drivers,
        const tap::algorithms::SmoothPidConfig& pidParams,
        float agitatorGearRatio,
        tap::motor::MotorId agitatorMotorId,
        tap::can::CanBus agitatorCanBusId,
        bool isAgitatorInverted,
        float jammingDistance,
        uint32_t jammingTime,
        bool jamLogicEnabled,
        const aruwsrc::control::launcher::FrictionWheelSubsystem* frictionWheelSubsystem,
        float launchSpeedThreshold);

    void refresh() override;

private:
    const aruwsrc::control::launcher::FrictionWheelSubsystem* frictionWheelSubsystem;

    /**
     * The minimum launch speed required to allow the agitator to move.
     */
    float launchSpeedThreshold;

};  // class LaunchSpeedGatedAgitator
}  // namespace aruwsrc::control::agitator

#endif  // LAUNCH_SPEED_GATED_AGITATOR_HPP_
