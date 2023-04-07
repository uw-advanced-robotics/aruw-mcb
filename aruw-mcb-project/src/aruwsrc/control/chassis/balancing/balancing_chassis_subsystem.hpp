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

#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"

#include "balancing_leg.hpp"

namespace aruwsrc::chassis
{

class BalancingChassisSubsystem : public tap::control::Subsystem
{
public:
    BalancingChassisSubsystem(tap::Drivers* drivers, BalancingLeg& leftLeg, BalancingLeg& rightLeg);

    void initialize() override;

    void refresh() override;

    void runHardwareTests() override;

    const char* getName() override { return "Balancing Chassis Subsystem"; }

    void setDesiredHeight(float z)
    {
        desiredZ += z;
        tap::algorithms::limitVal<float>(desiredZ, -.35, -.15);
    };

    void setDesiredOutput(float x, float r)
    {
        desiredX = x;
        desiredR = r;
    };

    void limitChassisPower();

private:
    void computeState();

    BalancingLeg &leftLeg, rightLeg;
    const tap::algorithms::SmoothPidConfig jankBalPidConfig{
        .kp = 25,
        .ki = .3,
        .kd = 0,
        .maxOutput = 200,
    };
    tap::algorithms::SmoothPid jankBalPid = tap::algorithms::SmoothPid(jankBalPidConfig);

    const tap::algorithms::SmoothPidConfig jankBalVelPidConfig{
        .kp = .1,
        .ki = 0,
        .kd = 0,
        .maxOutput = 15 * M_TWOPI / 360,
        .errDeadzone = 0,
    };
    tap::algorithms::SmoothPid jankBalVelPid = tap::algorithms::SmoothPid(jankBalVelPidConfig);

    float pitchAdjustment = 0;
    float pitchAdjustmentPrev = 0;
    float velocityAdjustment = 0;
    float velocityAdjustmentPrev = 0;
    float targetPitch; 

    float pitch;
    float roll;
    float yaw;

    float desiredX, desiredR, desiredZ;
    float currentX, currentV, currentR, currentZ;
    uint32_t prevTime;
};
}  // namespace aruwsrc::chassis

#endif  // BALANCING_CHASSIS_SUBSYSTEM_HPP_
