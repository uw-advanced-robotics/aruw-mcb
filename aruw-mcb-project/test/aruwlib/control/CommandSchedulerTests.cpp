/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include <aruwlib/Drivers.hpp>
#include <aruwlib/control/KillAllCommand.hpp>
#include <aruwlib/control/KillAllSubsystem.hpp>
#include <aruwlib/control/command_scheduler.hpp>
#include <gtest/gtest.h>

#include "aruwlib/mock/CommandMock.hpp"
#include "aruwlib/mock/SubsystemMock.hpp"

using aruwlib::Drivers;
using namespace aruwlib::control;

TEST(CommandScheduler, enterKillMode_do_not_enter_kill_mode_if_command_nullptr)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers);

    scheduler.enterKillMode(nullptr);
    EXPECT_FALSE(scheduler.getInKillMode());
}

TEST(CommandScheduler, enterKillMode_do_not_enter_kill_mode_if_command_not_in_scheduler)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers);
    KillAllSubsystem sub(&drivers);
    KillAllCommand cmd(&sub, &drivers);

    scheduler.enterKillMode(&cmd);
    EXPECT_FALSE(scheduler.getInKillMode());
}

TEST(CommandScheduler, enterKillMode_enter_kill_mode_if_command_in_scheduler)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers);
    KillAllSubsystem sub(&drivers);
    KillAllCommand cmd(&sub, &drivers);

    scheduler.registerSubsystem(&sub);
    scheduler.addCommand(&cmd);
    scheduler.enterKillMode(&cmd);
    EXPECT_TRUE(scheduler.getInKillMode());
}

TEST(CommandScheduler, enterKillMode_enter_kill_mode_calls_all_subsystems_onEnterKillMode_func)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers);
    KillAllSubsystem sub(&drivers);
    KillAllCommand cmd(&sub, &drivers);
    SubsystemMock subMock1(&drivers);
    SubsystemMock subMock2(&drivers);
    EXPECT_CALL(subMock1, onEnterKillMode).Times(1);
    EXPECT_CALL(subMock2, onEnterKillMode).Times(1);

    scheduler.registerSubsystem(&sub);
    scheduler.registerSubsystem(&subMock1);
    scheduler.registerSubsystem(&subMock2);
    scheduler.addCommand(&cmd);
    scheduler.enterKillMode(&cmd);
}

TEST(CommandScheduler, enterKillMode_enter_kill_mode_removes_all_commands)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers);
    KillAllSubsystem sub(&drivers);
    KillAllCommand cmd(&sub, &drivers);
    SubsystemMock subMock1(&drivers);
    SubsystemMock subMock2(&drivers);
    CommandMock cmdMock1;
    CommandMock cmdMock2;
    std::set<Subsystem *> cmd1Requirements = {&subMock1};
    std::set<Subsystem *> cmd2Requirements = {&subMock2};

    EXPECT_CALL(subMock1, onEnterKillMode).Times(1);
    EXPECT_CALL(subMock2, onEnterKillMode).Times(1);
    EXPECT_CALL(cmdMock1, initialize).Times(1);
    EXPECT_CALL(cmdMock2, initialize).Times(1);
    EXPECT_CALL(cmdMock1, end).Times(1);
    EXPECT_CALL(cmdMock2, end).Times(1);
    EXPECT_CALL(cmdMock1, getRequirements)
        .Times(1)
        .WillRepeatedly(testing::ReturnRef(cmd1Requirements));
    EXPECT_CALL(cmdMock2, getRequirements)
        .Times(1)
        .WillRepeatedly(testing::ReturnRef(cmd2Requirements));

    scheduler.registerSubsystem(&sub);
    scheduler.registerSubsystem(&subMock1);
    scheduler.registerSubsystem(&subMock2);
    scheduler.addCommand(&cmdMock1);
    scheduler.addCommand(&cmdMock2);
    scheduler.addCommand(&cmd);
    scheduler.enterKillMode(&cmd);
    EXPECT_FALSE(scheduler.isCommandScheduled(&cmdMock1));
    EXPECT_FALSE(scheduler.isCommandScheduled(&cmdMock2));
}

TEST(CommandScheduler, run_in_kill_mode_onRefreshKillMode_called_refresh_not_called)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers);
    KillAllSubsystem sub(&drivers);
    KillAllCommand cmd(&sub, &drivers);
    SubsystemMock subMock1(&drivers);
    SubsystemMock subMock2(&drivers);

    EXPECT_CALL(subMock1, onEnterKillMode).Times(1);
    EXPECT_CALL(subMock1, onRefreshKillMode).Times(1);
    EXPECT_CALL(subMock1, refresh).Times(0);
    EXPECT_CALL(subMock2, onEnterKillMode).Times(1);
    EXPECT_CALL(subMock2, onRefreshKillMode).Times(1);
    EXPECT_CALL(subMock2, refresh).Times(0);

    scheduler.registerSubsystem(&sub);
    scheduler.registerSubsystem(&subMock1);
    scheduler.registerSubsystem(&subMock2);
    scheduler.addCommand(&cmd);
    scheduler.enterKillMode(&cmd);
    scheduler.run();
}

TEST(CommandScheduler, addCommand_in_kill_mode_always_fails_to_add_command)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers);
    KillAllSubsystem sub(&drivers);
    KillAllCommand cmd(&sub, &drivers);
    SubsystemMock subMock(&drivers);
    CommandMock cmdMock;
    EXPECT_CALL(subMock, onEnterKillMode).Times(1);
    EXPECT_CALL(cmdMock, initialize).Times(0);

    scheduler.registerSubsystem(&sub);
    scheduler.registerSubsystem(&subMock);
    scheduler.addCommand(&cmd);
    scheduler.enterKillMode(&cmd);
    scheduler.addCommand(&cmdMock);
}

TEST(CommandScheduler, exitKillMode_calls_all_subsystems_onExitKillMode_function)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers);
    KillAllSubsystem sub(&drivers);
    KillAllCommand cmd(&sub, &drivers);
    SubsystemMock subMock1(&drivers);
    SubsystemMock subMock2(&drivers);
    EXPECT_CALL(subMock1, onEnterKillMode).Times(1);
    EXPECT_CALL(subMock1, onExitKillMode).Times(1);
    EXPECT_CALL(subMock2, onEnterKillMode).Times(1);
    EXPECT_CALL(subMock2, onExitKillMode).Times(1);

    scheduler.registerSubsystem(&sub);
    scheduler.registerSubsystem(&subMock1);
    scheduler.registerSubsystem(&subMock2);
    scheduler.addCommand(&cmd);
    scheduler.enterKillMode(&cmd);
    scheduler.exitKillMode();
}

TEST(CommandScheduler, exitKillMode_commands_may_be_added_after_exiting)
{
    Drivers drivers;
    CommandScheduler scheduler(&drivers);
    KillAllSubsystem sub(&drivers);
    KillAllCommand cmd(&sub, &drivers);
    SubsystemMock subMock1(&drivers);
    SubsystemMock subMock2(&drivers);
    CommandMock cmdMock1;
    CommandMock cmdMock2;
    std::set<Subsystem *> cmd1Requirements = {&subMock1};
    std::set<Subsystem *> cmd2Requirements = {&subMock2};

    EXPECT_CALL(cmdMock1, getRequirements)
        .Times(1)
        .WillRepeatedly(testing::ReturnRef(cmd1Requirements));
    EXPECT_CALL(cmdMock2, getRequirements)
        .Times(1)
        .WillRepeatedly(testing::ReturnRef(cmd2Requirements));
    EXPECT_CALL(cmdMock1, initialize).Times(1);
    EXPECT_CALL(cmdMock2, initialize).Times(1);
    EXPECT_CALL(subMock1, onEnterKillMode).Times(1);
    EXPECT_CALL(subMock1, onExitKillMode).Times(1);
    EXPECT_CALL(subMock2, onEnterKillMode).Times(1);
    EXPECT_CALL(subMock2, onExitKillMode).Times(1);

    scheduler.registerSubsystem(&sub);
    scheduler.registerSubsystem(&subMock1);
    scheduler.registerSubsystem(&subMock2);
    scheduler.addCommand(&cmd);
    scheduler.enterKillMode(&cmd);
    scheduler.exitKillMode();
    scheduler.addCommand(&cmdMock1);
    scheduler.addCommand(&cmdMock2);
}
