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

#ifndef FIVE_BAR_MOTION_SUBSYSTEM_HPP_
#define FIVE_BAR_MOTION_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/util_macros.hpp"

#include "aruwsrc/control/motion/five_bar_linkage.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::motion
{
enum MOTION_FUNCTIONS
{
    UP_AND_DOWN,
    SQUARE,
    CIRCLE,
    RETURN_TO_HOME,
};
static constexpr float MOTION_SIZE = .100;
/**
 * Testing motion system for FiveBar mechanism
 */
class FiveBarMotionSubsystem : public tap::control::Subsystem
{
public:
    /**
     * Creates a new FiveBar motion subsystem
     */
    FiveBarMotionSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorInterface* motor1,
        tap::motor::MotorInterface* motor2,
        FiveBarConfig fiveBarConfig,
        tap::algorithms::SmoothPidConfig motorPidConfig);

    void initialize() override;

    void refresh() override;

    void updateDesiredPosition(modm::Vector2f desiredPosition)
    {
        fiveBarLinkage.setDesiredPosition(desiredPosition);
    }

    void setMotionFunction(aruwsrc::control::motion::MOTION_FUNCTIONS func) {
        movementMode = func;
    }

    void onHardwareTestStart() { fiveBarLinkage.initialize(); }

    const char* getName() override { return "FiveBar"; }

private:

    modm::Vector2f pathPlotSquare(uint32_t time);
    modm::Vector2f pathPlotUpDown(uint32_t time);
    modm::Vector2f pathPlotCircle(uint32_t time);

    FiveBarLinkage fiveBarLinkage;

    MOTION_FUNCTIONS movementMode = UP_AND_DOWN;

    uint32_t prevZeroTime = 0;
};

}  // namespace aruwsrc::control::motion

#endif  // FRICTION_WHEEL_SUBSYSTEM_HPP_
