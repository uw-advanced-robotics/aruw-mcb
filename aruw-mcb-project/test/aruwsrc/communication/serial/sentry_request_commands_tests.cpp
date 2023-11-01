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

#include "tap/drivers.hpp"

#include "aruwsrc/communication/serial/sentry_request_commands.hpp"
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

class NoMotionStrategyCommandTest : public SentryRequestCommandsTest
{
protected:
    NoMotionStrategyCommandTest() : cmd(subsystem) {}
    NoMotionStrategyCommand cmd;
};

TEST_F(NoMotionStrategyCommandTest, isReady_true) { EXPECT_TRUE(cmd.isReady()); }

TEST_F(NoMotionStrategyCommandTest, isFinished_true) { EXPECT_TRUE(cmd.isFinished()); }

TEST_F(NoMotionStrategyCommandTest, initialize_queues_command)
{
    EXPECT_CALL(subsystem, queueRequest(SentryRequestMessageType::NONE));
    cmd.initialize();
}

class GoToFriendlyBaseCommandTest : public SentryRequestCommandsTest
{
protected:
    GoToFriendlyBaseCommandTest() : cmd(subsystem) {}
    GoToFriendlyBaseCommand cmd;
};

TEST_F(GoToFriendlyBaseCommandTest, isReady_true) { EXPECT_TRUE(cmd.isReady()); }

TEST_F(GoToFriendlyBaseCommandTest, isFinished_true) { EXPECT_TRUE(cmd.isFinished()); }

TEST_F(GoToFriendlyBaseCommandTest, initialize_queues_command)
{
    EXPECT_CALL(subsystem, queueRequest(SentryRequestMessageType::GO_TO_FRIENDLY_BASE));
    cmd.initialize();
}

class GoToEnemyBaseCommandTest : public SentryRequestCommandsTest
{
protected:
    GoToEnemyBaseCommandTest() : cmd(subsystem) {}
    GoToEnemyBaseCommand cmd;
};

TEST_F(GoToEnemyBaseCommandTest, isReady_true) { EXPECT_TRUE(cmd.isReady()); }

TEST_F(GoToEnemyBaseCommandTest, isFinished_true) { EXPECT_TRUE(cmd.isFinished()); }

TEST_F(GoToEnemyBaseCommandTest, initialize_queues_command)
{
    EXPECT_CALL(subsystem, queueRequest(SentryRequestMessageType::GO_TO_ENEMY_BASE));
    cmd.initialize();
}

class GoToSupplierZoneCommandTest : public SentryRequestCommandsTest
{
protected:
    GoToSupplierZoneCommandTest() : cmd(subsystem) {}
    GoToSupplierZoneCommand cmd;
};

TEST_F(GoToSupplierZoneCommandTest, isReady_true) { EXPECT_TRUE(cmd.isReady()); }

TEST_F(GoToSupplierZoneCommandTest, isFinished_true) { EXPECT_TRUE(cmd.isFinished()); }

TEST_F(GoToSupplierZoneCommandTest, initialize_queues_command)
{
    EXPECT_CALL(subsystem, queueRequest(SentryRequestMessageType::GO_TO_FRIENDLY_SUPPLIER_ZONE));
    cmd.initialize();
}

class GoToEnemySupplierZoneCommandTest : public SentryRequestCommandsTest
{
protected:
    GoToEnemySupplierZoneCommandTest() : cmd(subsystem) {}
    GoToEnemySupplierZoneCommand cmd;
};

TEST_F(GoToEnemySupplierZoneCommandTest, isReady_true) { EXPECT_TRUE(cmd.isReady()); }

TEST_F(GoToEnemySupplierZoneCommandTest, isFinished_true) { EXPECT_TRUE(cmd.isFinished()); }

TEST_F(GoToEnemySupplierZoneCommandTest, initialize_queues_command)
{
    EXPECT_CALL(subsystem, queueRequest(SentryRequestMessageType::GO_TO_ENEMY_SUPPLIER_ZONE));
    cmd.initialize();
}

class GoToCenterPointCommandTest : public SentryRequestCommandsTest
{
protected:
    GoToCenterPointCommandTest() : cmd(subsystem) {}
    GoToCenterPointCommand cmd;
};

TEST_F(GoToCenterPointCommandTest, isReady_true) { EXPECT_TRUE(cmd.isReady()); }

TEST_F(GoToCenterPointCommandTest, isFinished_true) { EXPECT_TRUE(cmd.isFinished()); }

TEST_F(GoToCenterPointCommandTest, initialize_queues_command)
{
    EXPECT_CALL(subsystem, queueRequest(SentryRequestMessageType::GO_TO_CENTER_POINT));
    cmd.initialize();
}

class HoldFireCommandTest : public SentryRequestCommandsTest
{
protected:
    HoldFireCommandTest() : cmd(subsystem) {}
    HoldFireCommand cmd;
};

TEST_F(HoldFireCommandTest, isReady_true) { EXPECT_TRUE(cmd.isReady()); }

TEST_F(HoldFireCommandTest, isFinished_true) { EXPECT_TRUE(cmd.isFinished()); }

TEST_F(HoldFireCommandTest, initialize_queues_command)
{
    EXPECT_CALL(subsystem, queueRequest(SentryRequestMessageType::HOLD_FIRE));
    cmd.initialize();
}

class ToggleMovementCommandTest : public SentryRequestCommandsTest
{
protected:
    ToggleMovementCommandTest() : cmd(subsystem) {}
    ToggleMovementCommand cmd;
};

TEST_F(ToggleMovementCommandTest, isReady_true) { EXPECT_TRUE(cmd.isReady()); }

TEST_F(ToggleMovementCommandTest, isFinished_true) { EXPECT_TRUE(cmd.isFinished()); }

TEST_F(ToggleMovementCommandTest, initialize_queues_command)
{
    EXPECT_CALL(subsystem, queueRequest(SentryRequestMessageType::TOGGLE_MOVEMENT));
    cmd.initialize();
}

class ToggleBeybladeCommandTest : public SentryRequestCommandsTest
{
protected:
    ToggleBeybladeCommandTest() : cmd(subsystem) {}
    ToggleBeybladeCommand cmd;
};

TEST_F(ToggleBeybladeCommandTest, isReady_true) { EXPECT_TRUE(cmd.isReady()); }

TEST_F(ToggleBeybladeCommandTest, isFinished_true) { EXPECT_TRUE(cmd.isFinished()); }

TEST_F(ToggleBeybladeCommandTest, initialize_queues_command)
{
    EXPECT_CALL(subsystem, queueRequest(SentryRequestMessageType::TOGGLE_BEYBLADE));
    cmd.initialize();
}