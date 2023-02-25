/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include <gtest/gtest.h>

#include "aruwsrc/communication/serial/sentry_request_commands.hpp"
#include "tap/drivers.hpp"
#include "aruwsrc/mock/sentry_request_subsystem_mock.hpp"

using namespace testing;
using namespace aruwsrc::communication::serial;

class SentryRequestCommandsTest : public Test
{
protected:
    SentryRequestCommandsTest() : subsystem(&drivers) {}
    tap::Drivers drivers;
    aruwsrc::mock::SentryRequestSubsystemMock subsystem;
};

class ToggleDriveMovementCommandTest : public SentryRequestCommandsTest
{
protected:
    ToggleDriveMovementCommandTest() : cmd(subsystem) {}
    ToggleDriveMovementCommand cmd;
};

class SelectNewRobotCommandTest : public SentryRequestCommandsTest
{
protected:
    SelectNewRobotCommandTest() : cmd(subsystem) {}
    SelectNewRobotCommand cmd;
};

class TargetNewQuadrantCommandTest : public SentryRequestCommandsTest
{
protected:
    TargetNewQuadrantCommandTest() : cmd(subsystem) {}
    TargetNewQuadrantCommand cmd;
};

TEST_F(ToggleDriveMovementCommandTest, isReady_true) { EXPECT_TRUE(cmd.isReady()); }

TEST_F(ToggleDriveMovementCommandTest, isFinished_true) { EXPECT_TRUE(cmd.isFinished()); }

TEST_F(ToggleDriveMovementCommandTest, initialize_queues_command)
{
    EXPECT_CALL(subsystem, queueRequest(SentryRequestMessageType::TOGGLE_DRIVE_MOVEMENT));
    cmd.initialize();
}

TEST_F(SelectNewRobotCommandTest, isReady_true) { EXPECT_TRUE(cmd.isReady()); }

TEST_F(SelectNewRobotCommandTest, isFinished_true) { EXPECT_TRUE(cmd.isFinished()); }

TEST_F(SelectNewRobotCommandTest, initialize_queues_command)
{
    EXPECT_CALL(subsystem, queueRequest(SentryRequestMessageType::SELECT_NEW_ROBOT));
    cmd.initialize();
}

TEST_F(TargetNewQuadrantCommandTest, isReady_true) { EXPECT_TRUE(cmd.isReady()); }

TEST_F(TargetNewQuadrantCommandTest, isFinished_true) { EXPECT_TRUE(cmd.isFinished()); }

TEST_F(TargetNewQuadrantCommandTest, initialize_queues_command)
{
    EXPECT_CALL(subsystem, queueRequest(SentryRequestMessageType::TARGET_NEW_QUADRANT));
    cmd.initialize();
}
