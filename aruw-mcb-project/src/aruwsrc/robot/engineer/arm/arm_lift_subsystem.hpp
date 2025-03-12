/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ARM_LIFT_SUBSYSTEM_HPP_
#define ARM_LIFT_SUBSYSTEM_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/motor_interface.hpp"
#include "tap/util_macros.hpp"

#include "aruwsrc/robot/engineer/arm/linear_joint_interface.hpp"

namespace aruwsrc
{
namespace engineer
{
class ArmLiftSubsystem : public LinearJointInterface
{
private:
    tap::algorithms::SmoothPid pidPos, pidAlign;
    tap::motor::DjiMotor &motorLeft, &motorRight;
    float radius;
    float kS = 0;

public:
    ArmLiftSubsystem(
        tap::Drivers *drivers,
        tap::motor::DjiMotor &motorLeft,
        tap::motor::DjiMotor &motorRight,
        tap::algorithms::SmoothPidConfig &config,
        float radius,
        float minSetpoint,
        float maxSetpoint,
        float kS = 0,
        float epsilon = 1);

    virtual float getPosition() override;

    float getPositionDifference();

    float getAverageVelocity();

    float getVelocityDifference();

    virtual bool atSetpoint() override;

    virtual void refresh() override;

    virtual void refreshSafeDisconnect() override;
};

}  // namespace engineer
}  // namespace aruwsrc

#endif  // ARM_LIFT_SUBSYSTEM_HPP_
