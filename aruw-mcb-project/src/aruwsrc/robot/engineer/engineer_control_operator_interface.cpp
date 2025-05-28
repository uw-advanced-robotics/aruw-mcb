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

#include "aruwsrc/robot/engineer/engineer_control_operator_interface.hpp"

namespace aruwsrc::control::engineer
{
bool EngineerControlOperatorInterface::isDriveMode()
{
    Remote::SwitchState leftState = drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH);
    return leftState == Remote::SwitchState::DOWN;
}

bool EngineerControlOperatorInterface::isGantryControlMode()
{
    Remote::SwitchState leftState = drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH);
    return leftState == Remote::SwitchState::MID;
}

bool EngineerControlOperatorInterface::isWristControlMode()
{
    Remote::SwitchState leftState = drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH);
    return leftState == Remote::SwitchState::UP;
}

float EngineerControlOperatorInterface::getCubeLiftVelocity()
{
    if (isGantryControlMode())
        return drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL);  // todo
    else
        return 0.0f;
}

float EngineerControlOperatorInterface::getArmLiftVelocity()
{
    if (isGantryControlMode())
        return drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL);
    else
        return 0.0f;
}

float EngineerControlOperatorInterface::getArmExtensionVelocity()
{
    if (isGantryControlMode())
        return drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL);
    else
        return 0.0f;
}

float EngineerControlOperatorInterface::getArmWristPitchVelocity()
{
    if (isWristControlMode())
        return drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL);
    else
        return 0.0f;
}

float EngineerControlOperatorInterface::getArmWristYawVelocity()
{
    if (isWristControlMode())
        return drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL);
    else
        return 0.0f;
}

float EngineerControlOperatorInterface::getArmWristRollVelocity()
{
    if (isWristControlMode())
        return drivers->remote.getChannel(Remote::Channel::WHEEL);
    else
        return 0.0f;
}

float EngineerControlOperatorInterface::getChassisXInput()
{
    if (isDriveMode())
    {
        return ControlOperatorInterface::getChassisXInput();
    }
    else
    {
        return 0.0f;
    }
}

float EngineerControlOperatorInterface::getChassisYInput()
{
    if (isDriveMode())
    {
        return ControlOperatorInterface::getChassisYInput();
    }
    else
    {
        return 0.0f;
    }
}

float EngineerControlOperatorInterface::getChassisRInput()
{
    if (isDriveMode())
    {
        return ControlOperatorInterface::getChassisRInput();
    }
    else
    {
        return 0.0f;
    }
}

}  // namespace aruwsrc::control::engineer
