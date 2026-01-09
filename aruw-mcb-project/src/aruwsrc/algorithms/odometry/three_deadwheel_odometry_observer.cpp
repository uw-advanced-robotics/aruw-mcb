/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "three_deadwheel_odometry_observer.hpp"

namespace aruwsrc::algorithms::odometry
{
ThreeDeadwheelOdometryObserver::ThreeDeadwheelOdometryObserver(
    tap::encoder::EncoderInterface* parallelWheelOne,
    tap::encoder::EncoderInterface* parallelWheelTwo,
    tap::encoder::EncoderInterface* perpendicularWheel,
    const float wheelRadius)
    : wheelRadius(wheelRadius),
      parallelWheelOne(parallelWheelOne),
      parallelWheelTwo(parallelWheelTwo),
      perpendicularWheel(perpendicularWheel)
{
}

float ThreeDeadwheelOdometryObserver::getParallelMotorOneVelocity() const
{
    return parallelWheelOne->getVelocity() * wheelRadius;
}
float ThreeDeadwheelOdometryObserver::getParallelMotorTwoVelocity() const
{
    return parallelWheelTwo->getVelocity() * wheelRadius;
}
float ThreeDeadwheelOdometryObserver::getPerpendicularVelocity() const
{
    return perpendicularWheel->getVelocity() * wheelRadius;
}

}  // namespace aruwsrc::algorithms::odometry
