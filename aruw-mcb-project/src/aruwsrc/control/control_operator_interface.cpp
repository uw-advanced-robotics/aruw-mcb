/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "control_operator_interface.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"

#include "aruwsrc/drivers.hpp"

using namespace tap::algorithms;
using namespace tap::communication::serial;

namespace aruwsrc
{
namespace control
{
float ControlOperatorInterface::getChassisXInput()
{
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();

    if (prevUpdateCounterX != updateCounter)
    {
        chassisXInput.update(drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL), currTime);
        prevUpdateCounterX = updateCounter;
    }

    int16_t input =
        drivers->remote.keyPressed(Remote::Key::W) - drivers->remote.keyPressed(Remote::Key::S);

    // Note for readability: chassisXKeyInputFiltered = The most recently filtered value computed by
    // this function (which we update below)
    if (abs(chassisXKeyInputFiltered) < CHASSIS_X_KEY_INPUT_FILTER_CHANGE_THRESHOLD ||
        abs(input) <= abs(chassisXKeyInputFiltered))
    {
        chassisXKeyInputFiltered =
            lowPassFilter(chassisXKeyInputFiltered, input, CHASSIS_X_KEY_INPUT_FILTER_ALPHA_MAX);
    }
    else
    {
        chassisXKeyInputFiltered = lowPassFilter(
            chassisXKeyInputFiltered,
            input,
            abs(chassisXKeyInputFiltered / input) * CHASSIS_X_KEY_INPUT_FILTER_ALPHA_MAX);
    }

    float finalX = limitVal<float>(
        chassisXInput.getInterpolatedValue(currTime) + chassisXKeyInputFiltered,
        -1.0f,
        1.0f);

    if (drivers->remote.keyPressed(Remote::Key::CTRL))
    {
        finalX *= CTRL_SCALAR;
    }
    if (drivers->remote.keyPressed(Remote::Key::SHIFT))
    {
        finalX *= SHIFT_SCALAR;
    }

    return finalX;
}

float ControlOperatorInterface::getChassisYInput()
{
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    if (prevUpdateCounterY != updateCounter)
    {
        chassisYInput.update(
            drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL),
            currTime);
        prevUpdateCounterY = updateCounter;
    }

    int16_t input =
        drivers->remote.keyPressed(Remote::Key::D) - drivers->remote.keyPressed(Remote::Key::A);

    // Note for readability: chassisYKeyInputFiltered = The most recently filtered value computed by
    // this function (which we update below)
    if (abs(chassisYKeyInputFiltered) < CHASSIS_Y_KEY_INPUT_FILTER_CHANGE_THRESHOLD ||
        abs(input) <= abs(chassisYKeyInputFiltered))
    {
        chassisYKeyInputFiltered =
            lowPassFilter(chassisYKeyInputFiltered, input, CHASSIS_Y_KEY_INPUT_FILTER_ALPHA_MAX);
    }
    else
    {
        chassisYKeyInputFiltered = lowPassFilter(
            chassisYKeyInputFiltered,
            input,
            abs(chassisYKeyInputFiltered / input) * CHASSIS_Y_KEY_INPUT_FILTER_ALPHA_MAX);
    }

    float finalY = limitVal<float>(
        chassisYInput.getInterpolatedValue(currTime) + chassisYKeyInputFiltered,
        -1.0f,
        1.0f);

    if (drivers->remote.keyPressed(Remote::Key::CTRL))
    {
        finalY *= CTRL_SCALAR;
    }
    if (drivers->remote.keyPressed(Remote::Key::SHIFT))
    {
        finalY *= SHIFT_SCALAR;
    }

    return finalY;
}

float ControlOperatorInterface::getChassisRInput()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    if (prevUpdateCounterR != updateCounter)
    {
        chassisRInput.update(
            drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL),
            currTime);
        prevUpdateCounterR = updateCounter;
    }

    chassisRKeyInputFiltered = lowPassFilter(
        chassisRKeyInputFiltered,
        drivers->remote.keyPressed(Remote::Key::Q) - drivers->remote.keyPressed(Remote::Key::E),
        CHASSIS_R_KEY_INPUT_FILTER_ALPHA);

    return limitVal<float>(
        chassisRInput.getInterpolatedValue(currTime) + chassisRKeyInputFiltered,
        -1.0f,
        1.0f);
}

float ControlOperatorInterface::getTurretYawInput()
{
    return -drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) +
           static_cast<float>(limitVal<int16_t>(
               -drivers->remote.getMouseX(),
               -USER_MOUSE_YAW_MAX,
               USER_MOUSE_YAW_MAX)) *
               USER_MOUSE_YAW_SCALAR;
}

float ControlOperatorInterface::getTurretPitchInput()
{
    return -drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL) +
           static_cast<float>(limitVal<int16_t>(
               drivers->remote.getMouseY(),
               -USER_MOUSE_PITCH_MAX,
               USER_MOUSE_PITCH_MAX)) *
               USER_MOUSE_PITCH_SCALAR;
}

float ControlOperatorInterface::getSentinelSpeedInput()
{
    return drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL) *
           USER_STICK_SENTINEL_DRIVE_SCALAR;
}
}  // namespace control

}  // namespace aruwsrc
