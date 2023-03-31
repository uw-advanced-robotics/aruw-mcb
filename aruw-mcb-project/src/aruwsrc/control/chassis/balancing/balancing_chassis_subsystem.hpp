/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef BALANCING_CHASSIS_SUBSYSTEM_HPP_
#define BALANCING_CHASSIS_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "balancing_leg.hpp"

namespace aruwsrc::chassis
{

class BalancingChassisSubsystem : public tap::control::Subsystem
{
public:
    BalancingChassisSubsystem(
        tap::Drivers* drivers,
        BalancingLeg& leftLeg,
        BalancingLeg& rightLeg);

    void initialize() override;

    void refresh() override;

    void runHardwareTests() override;

    const char* getName() override { return "Balancing Chassis Subsystem"; }

    void setDesiredHeight(float z);

    void setDesiredOutput(float x, float y, float r);

    void limitChassisPower();

private:
    BalancingLeg& leftLeg, rightLeg;
};
}   // namespace aruwsrc::chassis

#endif  // BALANCING_CHASSIS_SUBSYSTEM_HPP_
