/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef NON_SWERVE_WHEEL_HPP_
#define NON_SWERVE_WHEEL_HPP_

#include "wheel.hpp"

namespace aruwsrc
{
namespace chassis
{
/**
 * Encapsulates a non swerve wheel
 */

class NonSwerveWheel : public Wheel
{
public:
    NonSwerveWheel(Motor& driveMotor, float rollerAngle, WheelConfig& config);

    modm::Pair<float, float> calculateDesiredWheelVelocity(float vx, float vy, float vr);

    void executeWheelVelocity(float vx, float vy);

private:
    float const rollerAngle;
}; // class NonSwerveWheel

}  // namespace chassis
}  // namespace aruwsrc

#endif  // NON_SWERVE_WHEEL_HPP_
