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

#include "balstd_chassis_subsystem.hpp"

using namespace tap::algorithms;

namespace aruwsrc::control::balstd
{
BalstdChassisSubsystem::BalstdChassisSubsystem(
    tap::Drivers* drivers,
    BalstdLeg& leftLeg,
    BalstdLeg& rightLeg)
    : ChassisSubsystemInterface(drivers),
      leftLeg(leftLeg),
      rightLeg(rightLeg),
      controller(nullptr),
      currState(ZERO_STATE)
{
}

void BalstdChassisSubsystem::initialize()
{
    leftLeg.initialize();
    rightLeg.initialize();
}

bool BalstdChassisSubsystem::allMotorsOnline() const
{
    return leftLeg.allMotorsOnline() && rightLeg.allMotorsOnline();
}

void BalstdChassisSubsystem::setZeroRPM() { setOutputs(ZERO_OUTPUT); }

void BalstdChassisSubsystem::refresh()
{
    updateState();

    BalstdChassisOutput output =
        (controller == nullptr) ? ZERO_OUTPUT : controller->runController(currState);

    setOutputs(output);
}

void BalstdChassisSubsystem::setOutputs(const BalstdChassisOutput& output)
{
    leftLeg.setThrust(output.leftForce);
    leftLeg.setWheelTorque(output.leftTorque);
    rightLeg.setThrust(output.rightForce);
    rightLeg.setWheelTorque(output.rightTorque);
}

void BalstdChassisSubsystem::updateState()
{
    leftLeg.updateState();
    rightLeg.updateState();

    currState.leftLegState = leftLeg.getState();
    currState.rightLegState = rightLeg.getState();
}

}  // namespace aruwsrc::control::balstd
