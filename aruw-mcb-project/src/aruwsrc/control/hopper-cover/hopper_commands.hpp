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

#ifndef HOPPER_COMMANDS_HPP_
#define HOPPER_COMMANDS_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/setpoint/commands/move_absolute_command.hpp"

namespace aruwsrc
{
// forward declaration
namespace agitator
{
    class AgitatorSubsystem;
}
namespace control
{
/**
 * For opening and closing the hopper lid currently we use overly large
 * target angles for both opening and closing, this way (assuming no
 * interruption) the lid will keep moving until it hits the end of its
 * range of motion and "jams". Then we tell the motor to stop trying to
 * move and set it there. With this approach regardless of disconnects
 * and recalibrations we can hope to utilize the full hopper range of motion.
 */
class SoldierOpenHopperCommand : public tap::control::setpoint::MoveAbsoluteCommand
{
public:
    // 3.12 revolutions
    static constexpr float SOLDIER_OPEN_HOPPER_TARGET_ANGLE = 3.12 * M_PI;
    // 5000 milliradians/second
    static constexpr uint32_t SOLDIER_OPEN_HOPPER_ANGULAR_SPEED = 5.0f * M_PI * 1000.0f;
    // Allowable error in radians within which motor will consider target angle reached.
    static constexpr float SOLDIER_OPEN_HOPPER_TOLERANCE = 0.05f;
    // Whether or not the command should clear jam on end (based on how we want this command
    // to work the answer is yes, so `true`)
    static constexpr bool SOLDIER_OPEN_HOPPER_CLEAR_JAM = true;
    // Whether or not the command should set subsystem setpoint to ideal target on end
    static constexpr bool SOLDIER_OPEN_HOPPER_SET_SETPOINT_TO_TARGET_ON_END = false;
    SoldierOpenHopperCommand(tap::control::setpoint::SetpointSubsystem* agitator)
        : tap::control::setpoint::MoveAbsoluteCommand(
              agitator,
              SOLDIER_OPEN_HOPPER_TARGET_ANGLE,
              SOLDIER_OPEN_HOPPER_ANGULAR_SPEED,
              SOLDIER_OPEN_HOPPER_TOLERANCE,
              SOLDIER_OPEN_HOPPER_CLEAR_JAM,
              SOLDIER_OPEN_HOPPER_SET_SETPOINT_TO_TARGET_ON_END)
    {
    }

    /**
     * By default, the open hopper cover subsystem should only naturally finish if there is
     * a motor failure.
     */
    bool isFinished() const override { return !setpointSubsystem->isOnline(); }
};  // class SoldierOpenHopperCommand

/**
 * Use overly large target angle to keep moving agitator in one direction until
 * it "jams" and then stop moving the motor.
 *
 * This command is default scheduled so it has a slight twist, so as to not
 * overheat motors once it "jams" (which we use for detecting end of range
 * of motion) we set desired position to current position and continuously
 * return `false` from isFinished.
 */
class SoldierCloseHopperCommand : public tap::control::setpoint::MoveAbsoluteCommand
{
public:
    // 0 revolutions
    static constexpr float SOLDIER_CLOSE_HOPPER_TARGET_ANGLE = 0;
    // 5000 milliradians/second
    static constexpr uint32_t SOLDIER_CLOSE_HOPPER_ANGULAR_SPEED = 5.0f * M_PI * 1000.0f;
    // Allowable error in radians within which motor will consider target angle reached.
    static constexpr float SOLDIER_CLOSE_HOPPER_TOLERANCE = 0.05f;
    // Whether or not the command should clear jam on end (based on how we want this command
    // to work the answer is yes, so `true`)
    static constexpr bool SOLDIER_CLOSE_HOPPER_CLEAR_JAM = true;
    // Whether or not the command should set subsystem setpoint to ideal target on end
    static constexpr bool SOLDIER_CLOSE_HOPPER_SET_SETPOINT_TO_TARGET_ON_END = false;

    SoldierCloseHopperCommand(tap::control::setpoint::SetpointSubsystem* agitator)
        : tap::control::setpoint::MoveAbsoluteCommand(
              agitator,
              SOLDIER_CLOSE_HOPPER_TARGET_ANGLE,
              SOLDIER_CLOSE_HOPPER_ANGULAR_SPEED,
              SOLDIER_CLOSE_HOPPER_TOLERANCE,
              SOLDIER_CLOSE_HOPPER_CLEAR_JAM,
              SOLDIER_CLOSE_HOPPER_SET_SETPOINT_TO_TARGET_ON_END)
    {
    }

    bool isReady() override
    {
        /*
         * By default this command is scheduled, so only add the command if
         * the setpoint and the target don't match
         */
        float currAngle = setpointSubsystem->getCurrentValue();
        return !tap::algorithms::compareFloatClose(
            currAngle,
            SOLDIER_CLOSE_HOPPER_TARGET_ANGLE,
            SOLDIER_CLOSE_HOPPER_TOLERANCE);
    }
};  // class SoldierCloseHopperCommand

}  // namespace control

}  // namespace aruwsrc

#endif  // HOPPER_COMMANDS_HPP_
