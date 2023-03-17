/*
 * Copyright (c) 2023-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include "aruwsrc/control/motion/five_bar_motion_subsystem.hpp"

#include "tap/architecture/clock.hpp"

namespace aruwsrc::control::motion
{
FiveBarMotionSubsystem::FiveBarMotionSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface* motor1,
    tap::motor::MotorInterface* motor2,
    FiveBarConfig fiveBarConfig,
    tap::algorithms::SmoothPidConfig motorPidConfig)
    : tap::control::Subsystem(drivers),
      fiveBarLinkage(FiveBarLinkage(motor1, motor2, fiveBarConfig, motorPidConfig))
{
}

void FiveBarMotionSubsystem::initialize()
{
    fiveBarLinkage.initialize();
    // setMotionFunction(aruwsrc::control::motion::RETURN_TO_HOME);
}

void FiveBarMotionSubsystem::refresh()
{
    uint32_t time = tap::arch::clock::getTimeMilliseconds();
    modm::Vector2f desiredPosition;
    switch (movementMode)
    {
        case (UP_AND_DOWN):
            desiredPosition = pathPlotUpDown(time);
            break;
        case (CIRCLE):
            desiredPosition = pathPlotCircle(time);
            break;
        case (SQUARE):
            desiredPosition = pathPlotSquare(time);
            break;
        default:
            desiredPosition = fiveBarLinkage.getDefaultPosition();
            break;
    }
    fiveBarLinkage.setDesiredPosition(desiredPosition);
    fiveBarLinkage.refresh();
    if (time - prevZeroTime > 4000)
    {
        prevZeroTime = time;
    }
}

modm::Vector2f FiveBarMotionSubsystem::pathPlotUpDown(uint32_t time)
{
    if ((time - prevZeroTime > 0) && (time - prevZeroTime <= 2000))
    {
        return modm::Vector2f(
            fiveBarLinkage.getDefaultPosition().getX(),
            fiveBarLinkage.getDefaultPosition().getY() - MOTION_SIZE);
    }
    else
    {
        return modm::Vector2f(
            fiveBarLinkage.getDefaultPosition().getX(),
            fiveBarLinkage.getDefaultPosition().getY());
    }
}

modm::Vector2f FiveBarMotionSubsystem::pathPlotCircle(uint32_t time)
{
    // move in a circle centered around MOTION_SIZE/2 below the default position
    // period should be 4 seconds
    float radius = MOTION_SIZE / 2;
    float theta = M_TWOPI * (time - prevZeroTime) / 4000;
    float xDesired = radius * cos(theta) + fiveBarLinkage.getDefaultPosition().getX();
    float yDesired = radius * sin(theta) + fiveBarLinkage.getDefaultPosition().getY() - radius;
    return modm::Vector2f(xDesired, yDesired);
}

modm::Vector2f FiveBarMotionSubsystem::pathPlotSquare(uint32_t time)
{
    uint32_t dt = time - prevZeroTime;
    float xDes = fiveBarLinkage.getDefaultPosition().getX();
    float yDes = fiveBarLinkage.getDefaultPosition().getY();
    if (dt > 0 && dt <= 1000)
    {  // top right
        xDes = fiveBarLinkage.getDefaultPosition().getX() + MOTION_SIZE / 2;
        yDes = fiveBarLinkage.getDefaultPosition().getY() - MOTION_SIZE / 4;
    }
    else if (dt > 1000 && dt <= 2000)
    {  // bottom right
        xDes = fiveBarLinkage.getDefaultPosition().getX() + MOTION_SIZE / 2;
        yDes = fiveBarLinkage.getDefaultPosition().getY() - MOTION_SIZE - MOTION_SIZE / 4;
    }
    else if (dt > 2000 && dt <= 3000)
    {  // bottom right
        xDes = fiveBarLinkage.getDefaultPosition().getX() - MOTION_SIZE / 2;
        yDes = fiveBarLinkage.getDefaultPosition().getY() - MOTION_SIZE - MOTION_SIZE / 4;
    }
    else if (dt > 3000 && dt <= 4000)
    {  // bottom right
        xDes = fiveBarLinkage.getDefaultPosition().getX() - MOTION_SIZE / 2;
        yDes = fiveBarLinkage.getDefaultPosition().getY() - MOTION_SIZE / 4;
    }
    return modm::Vector2f(xDes, yDes);
}
}  // namespace aruwsrc::control::motion