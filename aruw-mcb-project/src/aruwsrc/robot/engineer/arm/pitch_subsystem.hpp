/*
 * Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef PITCH_SUBSYSTEM_HPP_
#define PITCH_SUBSYSTEM_HPP_

#include "joint_subsystem.hpp"

namespace aruwsrc::engineer::arm
{
class PitchSubsystem : public JointSubsystem
{
public:
    PitchSubsystem(
        tap::Drivers* drivers,
        const JointSubsystemConfig& config,
        tap::motor::MotorInterface* motor)
        : JointSubsystem(drivers, config, motor, "Pitch joint")
    {
    }

    /**
     * This one is overriden to account for differing gravity feedforwards as the arm goes to
     * different setpoints. Basically, more feedforward as we are parallel to the ground.
     */
    void refresh() override
    {
        float setpointRadians = setpoint / config.setpointToEncoderScalar;
        float compensatedFeedforward = config.feedforward * cosf(setpointRadians);

        positionPid.update(setpoint - motor->getEncoderUnwrapped());
        motor->setDesiredOutput(positionPid.getValue() + compensatedFeedforward);
    }
};  // class PitchSubsystem

}  // namespace aruwsrc::engineer
#endif  // ARM_SUBSYSTEM_HPP_
