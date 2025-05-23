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
    BalstdLeg& rightLeg,
    tap::communication::sensors::imu::ImuInterface& chassisImu)
    : ChassisSubsystemInterface(drivers),
      leftLeg(leftLeg),
      rightLeg(rightLeg),
      chassisImu(chassisImu),
      controller(nullptr),
      currState(ZERO_STATE),
      currOutput(ZERO_OUTPUT)
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

    currOutput =
        (controller == nullptr) ? ZERO_OUTPUT : controller->runController(currState, 0.002f);

    setOutputs(currOutput);

    leftLeg.refresh();
    rightLeg.refresh();
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
    // todo: maybe move into dedicated observer class
    leftLeg.updateState();
    rightLeg.updateState();
    currState.leftLegState = leftLeg.getState();
    currState.rightLegState = rightLeg.getState();

    currState.virtualLegState.P3.data[0] =
        (currState.leftLegState.P3.data[0] + currState.rightLegState.P3.data[0]) / 2;
    currState.virtualLegState.P3.data[1] =
        (currState.leftLegState.P3.data[1] + currState.rightLegState.P3.data[1]) / 2;
    currState.virtualLegState.vxc = (currState.leftLegState.vxc + currState.rightLegState.vxc) / 2;
    currState.virtualLegState.vyc = (currState.leftLegState.vyc + currState.rightLegState.vyc) / 2;
    currState.virtualLegState.calculatePendulumState();

    currState.roll = -chassisImu.getRoll();
    currState.rollVel = -chassisImu.getGx();
    currState.pitch = -chassisImu.getPitch();
    currState.pitchVel = -chassisImu.getGy();
    currState.yaw = chassisImu.getYaw();
    currState.yawVel = chassisImu.getGz();

    currState.height = currState.virtualLegState.L * cos(currState.virtualLegState.alpha);

    // should be __ - pitch/vel, but matlab's pitch is positive up instead of down
    currState.virtualPendTheta = currState.virtualLegState.alpha + currState.pitch;
    currState.virtualPendThetaDot = currState.virtualLegState.alphaDot + currState.pitchVel;

    currState.virtualWheelVel =
        (currState.leftLegState.wheelVel + currState.rightLegState.wheelVel) / 2 * WHEEL_RADIUS_M;
    currState.virtualWheelPos += currState.virtualWheelVel * 0.002f;
}

}  // namespace aruwsrc::control::balstd
