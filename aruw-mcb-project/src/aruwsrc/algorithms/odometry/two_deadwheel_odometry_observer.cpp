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

#include "two_deadwheel_odometry_observer.hpp"

namespace aruwsrc::algorithms::odometry
{
TwoDeadwheelOdometryObserver::TwoDeadwheelOdometryObserver(
#ifdef TARGET_SENTRY_HYDRA
    aruwsrc::virtualMCB::VirtualDjiMotor* parallelWheel,
    aruwsrc::virtualMCB::VirtualDjiMotor* perpendicularWheel,
#else
    tap::encoder::EncoderInterface* parallelWheel,
    tap::encoder::EncoderInterface* perpendicularWheel,
#endif
    const float wheelRadius)
    : wheelRadius(wheelRadius),
      parallelWheel(parallelWheel),
      perpendicularWheel(perpendicularWheel)
{
}

float TwoDeadwheelOdometryObserver::getParallelMotorVelocity() const
{
#ifdef TARGET_SENTRY_HYDRA
    return parallelWheel->getEncoder()->getVelocity() * wheelRadius;
#else
    return parallelWheel->getVelocity() * wheelRadius;
#endif
}
float TwoDeadwheelOdometryObserver::getPerpendicularVelocity() const
{
#ifdef TARGET_SENTRY_HYDRA
    return perpendicularWheel->getEncoder()->getVelocity() * wheelRadius;
#else
    return perpendicularWheel->getVelocity() * wheelRadius;
#endif
}

}  // namespace aruwsrc::algorithms::odometry
