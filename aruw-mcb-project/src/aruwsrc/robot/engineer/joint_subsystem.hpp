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

#ifndef JOINT_SUBSYSTEM_HPP_
#define JOINT_SUBSYSTEM_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/motor_interface.hpp"
#include "tap/util_macros.hpp"

#include "aruwsrc/robot/engineer/linear_joint_interface.hpp"

namespace aruwsrc::engineer
{
/**
 * Subsystem code for rotational joints.
 */
class JointSubsystem : public LinearJointInterface, public tap::control::Subsystem
{
public:
    JointSubsystem(
        tap::Drivers *drivers,
        tap::motor::MotorInterface &motor,
        const tap::algorithms::SmoothPidConfig &config,
        float lowerBound = 0.0f,
        float upperBound = 0.0f,
        float kS = 0,
        float epsilon = 1e-4f);

    virtual void initialize() override;

    virtual float getPosition() override;

    virtual void refresh() override;

    virtual void refreshSafeDisconnect() override;

private:
    tap::algorithms::SmoothPid pid;
    tap::motor::MotorInterface &motor;
    // Constant added to output to overcome static friction
    float kS;
};

}  // namespace aruwsrc::engineer

#endif  // JOINT_SUBSYSTEM_HPP_