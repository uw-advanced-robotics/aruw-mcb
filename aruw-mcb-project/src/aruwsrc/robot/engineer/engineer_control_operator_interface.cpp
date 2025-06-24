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
    return drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::DOWN &&
           drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) != Remote::SwitchState::UP;
}

bool EngineerControlOperatorInterface::isGantryControlMode()
{
    return drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID &&
           drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) != Remote::SwitchState::UP;
}

bool EngineerControlOperatorInterface::isWristControlMode()
{
    return drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP &&
           drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) != Remote::SwitchState::UP;
}

float EngineerControlOperatorInterface::getCubeLiftVelocity()
{
    return drivers->remote.getChannel(Remote::Channel::WHEEL);
    // if (isGantryControlMode())
    //     return drivers->remote.getChannel(Remote::Channel::WHEEL);
    
}

float EngineerControlOperatorInterface::getGantryLiftVelocity()
{
    return drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL);
    // if (isGantryControlMode())
    //     return drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL);
  
}

float EngineerControlOperatorInterface::getGantryExtensionVelocity()
{
    if (isGantryControlMode()) {
        return drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL) + drivers->remote.getMouseX();
    } else {
        return drivers->remote.getMouseX();
    }
}

bool EngineerControlOperatorInterface::getGantryKeyUp() {
    return drivers->remote.keyPressed(Remote::Key::F);
}

bool EngineerControlOperatorInterface::getGantryKeyDown() {
    return drivers->remote.keyPressed(Remote::Key::V);
}

bool EngineerControlOperatorInterface::getGantryKeyIn() {
    return drivers->remote.keyPressed(Remote::Key::B);
}

bool EngineerControlOperatorInterface::getGantryKeyOut() {
    return drivers->remote.keyPressed(Remote::Key::G);
}

bool EngineerControlOperatorInterface::getShiftKey() {
    return drivers->remote.keyPressed(Remote::Key::SHIFT);
}

float EngineerControlOperatorInterface::getWristPitchVelocity()
{
    if (isWristControlMode()) {
        return drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) + drivers->remote.getMouseY();
    } else {
        return drivers->remote.getMouseY();
    }
}
float EngineerControlOperatorInterface::getWristYawVelocity()
{
    if (isWristControlMode()) {
        return drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL) + drivers->remote.getMouseX();
    } else {
        return drivers->remote.getMouseX();
    }
}

float wristRollVelocity = 0.5;
float EngineerControlOperatorInterface::getWristRollVelocity()
{
    if (isWristControlMode()) {
        return drivers->remote.getChannel(Remote::Channel::WHEEL);
    } else if (drivers->remote.getMouseL() && drivers->remote.keyPressed(Remote::Key::SHIFT)) {
        return -wristRollVelocity; //TODO: fix 
    } else {
        return wristRollVelocity;
    }
}

float EngineerControlOperatorInterface::getChassisXInput()
{
    float xInput =  ControlOperatorInterface::getChassisXInput();
    if(drivers->remote.keyPressed(Remote::Key::CTRL)) {
        return xInput;
    } else {
        return xInput / 2;
    }
}

float EngineerControlOperatorInterface::getChassisYInput()
{
    float yInput =  ControlOperatorInterface::getChassisYInput();
    if(drivers->remote.keyPressed(Remote::Key::CTRL)) {
        return yInput;
    } else {
        return yInput / 2;
    }
}

float EngineerControlOperatorInterface::getChassisRInput()
{
    float rInput =  ControlOperatorInterface::getChassisRInput();
    if(drivers->remote.keyPressed(Remote::Key::CTRL)) {
        return rInput;
    } else {
        return rInput / 2;
    }
}

}  // namespace aruwsrc::control::engineer
