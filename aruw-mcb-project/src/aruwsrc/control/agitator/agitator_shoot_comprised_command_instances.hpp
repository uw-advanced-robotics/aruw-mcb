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

#ifndef AGITATOR_SHOOT_COMPRISED_COMMAND_INSTANCES_HPP_
#define AGITATOR_SHOOT_COMPRISED_COMMAND_INSTANCES_HPP_

#include "tap/control/setpoint/commands/move_unjam_comprised_command.hpp"
#include "tap/control/setpoint/interfaces/setpoint_subsystem.hpp"
#include "tap/drivers.hpp"

#include "limit_switch_agitator_subsystem.hpp"

namespace aruwsrc
{
namespace agitator
{
class AgitatorSubsystem;

/**
 * A class that extends the shoot comprised command and defines the system parameters of the
 * comprised command. The constants are choosen for fast rotation speed for a soldier robot's
 * agitator.
 */
class MoveUnjamRefLimitedCommand : public tap::control::setpoint::MoveUnjamComprisedCommand
{
public:
    MoveUnjamRefLimitedCommand(
        tap::Drivers* drivers,
        AgitatorSubsystem* agitator17mm,
        float agitatorRotateAngle,
        float maxUnjamRotateAngle,
        uint32_t rotateTime,
        bool heatLimiting,
        float heatLimitBuffer);

    bool isReady() override;

    bool isFinished() const override;

private:
    tap::Drivers* drivers;

    const bool heatLimiting;
    const float heatLimitBuffer;
};  // class ShootFastComprisedCommand

/**
 * Command that rotates the associated waterwheel if there are less than some number
 * of balls in the tube between the waterwheel and the firing mechanism.
 */
class WaterwheelLoadCommand42mm : public tap::control::setpoint::MoveUnjamComprisedCommand
{
public:
    // Angle the command tries to move the agitator whenever it is scheduled
    static constexpr float WATERWHEEL_42MM_CHANGE_ANGLE = M_PI / 7;
    // Max angle the agitator will move while unjamming
    static constexpr float WATERWHEEL_42MM_MAX_UNJAM_ANGLE = M_PI / 35;
    // Expected time for the water wheel to rotate the specified angle in ms
    static constexpr uint32_t WATERWHEEL_42MM_ROTATE_TIME = 1000;
    // How long the command should wait after reaching the target angle
    static constexpr uint32_t WATERWHEEL_42MM_PAUSE_AFTER_ROTATE_TIME = 10;

    // Buffer from max heat limit in which limiting occurs, for hero 100 is one shot.
    static constexpr float HEAT_LIMIT_BUFFER = 100;

    static constexpr int BALLS_QUEUED_IN_TUBE = 3;

    WaterwheelLoadCommand42mm(
        tap::Drivers* drivers,
        aruwsrc::agitator::LimitSwitchAgitatorSubsystem* waterwheel);

    bool isReady() override;

    bool isFinished() const override;

private:
    // Store instance of drivers to be able to access digital
    tap::Drivers* drivers;

    // Store pointer to limited agitator subsystem with derived class type
    aruwsrc::agitator::LimitSwitchAgitatorSubsystem* waterwheel;
};  // class Waterwheel42mmLoadCommand

/**
 * This command rotates the kicker subsystem, no jamming required so just uses a MoveCommand
 */
class ShootCommand42mm : public tap::control::setpoint::MoveCommand
{
public:
    // Angle the command tries to move the agitator whenever it is scheduled
    static constexpr float KICKER_42MM_CHANGE_ANGLE = 1.3f * M_PI;
    // Expected time for the water wheel to rotate the specified angle in ms
    static constexpr uint32_t KICKER_42MM_ROTATE_TIME = 300;
    // How long the command should wait after reaching the target angle
    static constexpr uint32_t KICKER_42MM_PAUSE_AFTER_ROTATE_TIME = 0;

    // Buffer from max heat limit in which limiting occurs, for hero 100 is one shot.
    static constexpr uint16_t HEAT_LIMIT_BUFFER = 100;

    ShootCommand42mm(
        tap::Drivers* drivers,
        tap::control::setpoint::SetpointSubsystem* kicker,
        bool heatLimiting = true);

    // Override for heat limiting logic
    bool isReady() override;

    void initialize() override;

    static int getInitializeCount() { return initializeCount; }
    static void resetInitializeCount() { initializeCount = 0; }

private:
    tap::Drivers* drivers;

    const bool heatLimiting;

    static int initializeCount;
};

}  // namespace agitator

}  // namespace aruwsrc

#endif  // AGITATOR_SHOOT_COMPRISED_COMMAND_INSTANCES_HPP_
