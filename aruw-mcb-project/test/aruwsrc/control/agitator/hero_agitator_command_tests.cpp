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

#include <gtest/gtest.h>

#include "tap/mock/command_mock.hpp"
#include "tap/mock/integrable_setpoint_subsystem_mock.hpp"
#include "tap/mock/odometry_2d_interface_mock.hpp"

#include "aruwsrc/control/agitator/hero_agitator_command.hpp"
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/mock/friction_wheel_subsystem_mock.hpp"
#include "aruwsrc/mock/referee_feedback_friction_wheel_subsystem_mock.hpp"
#include "aruwsrc/mock/turret_cv_command_mock.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace aruwsrc;
using namespace aruwsrc::agitator;
using namespace aruwsrc::mock;
using namespace tap::communication::serial;
using namespace testing;

using tap::arch::clock::ClockStub;

static HeroAgitatorCommand::Config DEFAULT_HERO_AGITATOR_CMD_CONFIG = {
    .heatLimiting = false,
    .heatLimitBuffer = 100,
};

class CommandWithRequirementsMock : public tap::mock::CommandMock
{
public:
    CommandWithRequirementsMock(uint64_t requirementsBitwise) : tap::mock::CommandMock()
    {
        ON_CALL(*this, getRequirementsBitwise).WillByDefault(Return(requirementsBitwise));
    }
};

class HeroAgitatorCommandTest : public Test
{
protected:
    HeroAgitatorCommandTest()
        : drivers(),
          kicker(&drivers),
          waterwheel(&drivers),
          frictionWheels(&drivers),
          kickerFireCommand(1UL << kicker.getGlobalIdentifier()),
          kickerLoadCommand(1UL << kicker.getGlobalIdentifier()),
          waterwheelLoadCommand(1UL << waterwheel.getGlobalIdentifier()),
          turretCVCommand(&drivers, nullptr, nullptr, nullptr, odometry, frictionWheels, 0, 0, 0),
          cmd(drivers,
              DEFAULT_HERO_AGITATOR_CMD_CONFIG,
              kicker,
              waterwheel,
              frictionWheels,
              turretCVCommand,
              kickerFireCommand,
              kickerLoadCommand,
              waterwheelLoadCommand)
    {
    }

    void SetUp() override
    {
        ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));
        ON_CALL(turretCVCommand, isAimingWithinLaunchingTolerance).WillByDefault(Return(true));
    }

    Drivers drivers;
    NiceMock<tap::mock::IntegrableSetpointSubsystemMock> kicker;
    NiceMock<tap::mock::IntegrableSetpointSubsystemMock> waterwheel;
    NiceMock<RefereeFeedbackFrictionWheelSubsystemMock> frictionWheels;
    NiceMock<CommandWithRequirementsMock> kickerFireCommand;
    NiceMock<CommandWithRequirementsMock> kickerLoadCommand;
    NiceMock<CommandWithRequirementsMock> waterwheelLoadCommand;
    NiceMock<tap::mock::Odometry2DInterfaceMock> odometry;
    NiceMock<aruwsrc::mock::TurretCVCommandMock> turretCVCommand;
    HeroAgitatorCommand cmd;
    RefSerial::Rx::RobotData robotData;
    ClockStub clock;
};

TEST_F(HeroAgitatorCommandTest, isReady_no_heat_limiting_true_when_both_motors_online)
{
    ON_CALL(frictionWheels, getDesiredLaunchSpeed).WillByDefault(Return(20));
    ON_CALL(kicker, isOnline).WillByDefault(Return(true));
    ON_CALL(waterwheel, isOnline).WillByDefault(Return(true));

    EXPECT_TRUE(cmd.isReady());
}

TEST_F(HeroAgitatorCommandTest, isReady_no_heat_limiting_false_when_single_motor_offline)
{
    ON_CALL(frictionWheels, getDesiredLaunchSpeed).WillByDefault(Return(20));
    EXPECT_CALL(kicker, isOnline)
        .Times(AnyNumber())
        .WillOnce(Return(true))
        .WillOnce(Return(false))
        .WillOnce(Return(false));
    EXPECT_CALL(waterwheel, isOnline)
        .Times(AnyNumber())
        .WillOnce(Return(false))
        .WillOnce(Return(true))
        .WillOnce(Return(false));

    EXPECT_FALSE(cmd.isReady());
    EXPECT_FALSE(cmd.isReady());
    EXPECT_FALSE(cmd.isReady());
}

TEST_F(HeroAgitatorCommandTest, isReady_no_heat_limiting_true_when_heat_limit_within_buffer)
{
    ON_CALL(frictionWheels, getDesiredLaunchSpeed).WillByDefault(Return(20));
    ON_CALL(kicker, isOnline).WillByDefault(Return(true));
    ON_CALL(waterwheel, isOnline).WillByDefault(Return(true));

    robotData.turret.heatLimit42 = 200;
    robotData.turret.heat42 = 150;

    EXPECT_TRUE(cmd.isReady());
}

TEST_F(HeroAgitatorCommandTest, isReady_heat_limiting_true_when_heat_limit_below_buffer)
{
    auto agitatorConfig = DEFAULT_HERO_AGITATOR_CMD_CONFIG;
    agitatorConfig.heatLimiting = true;

    // declare new HeroAgitatorCommand that has a custom agitator config
    HeroAgitatorCommand cmd(
        drivers,
        agitatorConfig,
        kicker,
        waterwheel,
        frictionWheels,
        turretCVCommand,
        kickerFireCommand,
        kickerLoadCommand,
        waterwheelLoadCommand);

    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(true));
    ON_CALL(frictionWheels, getDesiredLaunchSpeed).WillByDefault(Return(20));
    ON_CALL(kicker, isOnline).WillByDefault(Return(true));
    ON_CALL(waterwheel, isOnline).WillByDefault(Return(true));

    robotData.turret.heatLimit42 = 100;
    robotData.turret.heat42 = 0;

    EXPECT_TRUE(cmd.isReady());

    robotData.turret.heatLimit42 = 200;
    robotData.turret.heat42 = 100;

    EXPECT_TRUE(cmd.isReady());
}

TEST_F(HeroAgitatorCommandTest, isReady_heat_limiting_false_when_heat_limit_above_buffer)
{
    auto agitatorConfig = DEFAULT_HERO_AGITATOR_CMD_CONFIG;
    agitatorConfig.heatLimiting = true;

    // declare new HeroAgitatorCommand that has a custom agitator config
    HeroAgitatorCommand cmd(
        drivers,
        agitatorConfig,
        kicker,
        waterwheel,
        frictionWheels,
        turretCVCommand,
        kickerFireCommand,
        kickerLoadCommand,
        waterwheelLoadCommand);

    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(true));
    ON_CALL(frictionWheels, getDesiredLaunchSpeed).WillByDefault(Return(20));
    ON_CALL(kicker, isOnline).WillByDefault(Return(true));
    ON_CALL(waterwheel, isOnline).WillByDefault(Return(true));

    robotData.turret.heatLimit42 = 100;
    robotData.turret.heat42 = 1;

    EXPECT_FALSE(cmd.isReady());

    robotData.turret.heatLimit42 = 200;
    robotData.turret.heat42 = 201;

    EXPECT_FALSE(cmd.isReady());

    robotData.turret.heatLimit42 = 200;
    robotData.turret.heat42 = 350;
    EXPECT_FALSE(cmd.isReady());
}

TEST_F(HeroAgitatorCommandTest, isReady_false_when_flywheels_not_on)
{
    ON_CALL(frictionWheels, getDesiredLaunchSpeed).WillByDefault(Return(0));
    ON_CALL(kicker, isOnline).WillByDefault(Return(true));
    ON_CALL(waterwheel, isOnline).WillByDefault(Return(true));

    EXPECT_FALSE(cmd.isReady());
}

TEST_F(HeroAgitatorCommandTest, isFinished_true_when_flywheels_not_on)
{
    ON_CALL(frictionWheels, getDesiredLaunchSpeed).WillByDefault(Return(0));
    ON_CALL(kicker, isOnline).WillByDefault(Return(true));
    ON_CALL(waterwheel, isOnline).WillByDefault(Return(true));

    EXPECT_TRUE(cmd.isFinished());
}

TEST_F(HeroAgitatorCommandTest, isFinished_true_when_motors_disconnected)
{
    ON_CALL(frictionWheels, getDesiredLaunchSpeed).WillByDefault(Return(20));

    bool kickerOnline = false;
    bool waterwheelOnline = false;
    ON_CALL(kicker, isOnline).WillByDefault(ReturnPointee(&kickerOnline));
    ON_CALL(waterwheel, isOnline).WillByDefault(ReturnPointee(&waterwheelOnline));

    EXPECT_TRUE(cmd.isFinished());

    waterwheelOnline = true;
    EXPECT_TRUE(cmd.isFinished());

    kickerOnline = true;
    waterwheelOnline = false;
    EXPECT_TRUE(cmd.isFinished());
}

TEST_F(HeroAgitatorCommandTest, execute_ball_not_loaded_loading_happens)
{
    bool limitSwitchDepressed = false;
    ON_CALL(drivers.turretMCBCanComm, getLimitSwitchDepressed)
        .WillByDefault(ReturnPointee(&limitSwitchDepressed));

    EXPECT_CALL(kickerLoadCommand, initialize).Times(2);
    EXPECT_CALL(waterwheelLoadCommand, initialize).Times(2);

    ON_CALL(kickerLoadCommand, isReady).WillByDefault(Return(true));
    ON_CALL(kickerLoadCommand, isFinished).WillByDefault(Return(true));
    ON_CALL(waterwheelLoadCommand, isReady).WillByDefault(Return(true));
    ON_CALL(waterwheelLoadCommand, isFinished).WillByDefault(Return(true));

    cmd.initialize();

    cmd.execute();
    cmd.execute();

    limitSwitchDepressed = true;

    cmd.execute();

    EXPECT_TRUE(cmd.isFinished());
}

TEST_F(HeroAgitatorCommandTest, execute_ball_not_loaded_multiple_load_cycles_happen)
{
    bool limitSwitchDepressed = false;
    ON_CALL(drivers.turretMCBCanComm, getLimitSwitchDepressed)
        .WillByDefault(ReturnPointee(&limitSwitchDepressed));

    EXPECT_CALL(kickerLoadCommand, initialize).Times(5);
    EXPECT_CALL(waterwheelLoadCommand, initialize).Times(5);

    ON_CALL(kickerLoadCommand, isReady).WillByDefault(Return(true));
    ON_CALL(kickerLoadCommand, isFinished).WillByDefault(Return(true));
    ON_CALL(waterwheelLoadCommand, isReady).WillByDefault(Return(true));
    ON_CALL(waterwheelLoadCommand, isFinished).WillByDefault(Return(true));

    cmd.initialize();

    cmd.execute();
    cmd.execute();
    cmd.execute();
    cmd.execute();
    cmd.execute();
}

TEST_F(
    HeroAgitatorCommandTest,
    execute_ready_to_fire_refserial_offline_firing_happens_then_loading_happens_when_limit_switch_not_depressed)
{
    bool limitSwitchDepressed = true;
    ON_CALL(drivers.turretMCBCanComm, getLimitSwitchDepressed)
        .WillByDefault(ReturnPointee(&limitSwitchDepressed));

    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(false));

    ON_CALL(frictionWheels, getDesiredLaunchSpeed).WillByDefault(Return(20));
    ON_CALL(kicker, isOnline).WillByDefault(Return(true));
    ON_CALL(waterwheel, isOnline).WillByDefault(Return(true));

    ON_CALL(kickerFireCommand, isReady).WillByDefault(Return(true));

    ON_CALL(waterwheelLoadCommand, isFinished).WillByDefault(Return(true));
    ON_CALL(kickerLoadCommand, isFinished).WillByDefault(Return(true));

    bool kickerFireCommandFinished = false;
    ON_CALL(kickerFireCommand, isFinished).WillByDefault(ReturnPointee(&kickerFireCommandFinished));

    EXPECT_CALL(kickerFireCommand, initialize);
    EXPECT_CALL(kickerLoadCommand, initialize);
    EXPECT_CALL(waterwheelLoadCommand, initialize);

    cmd.initialize();

    cmd.execute();

    // still loading since command not finished
    EXPECT_FALSE(cmd.isFinished());

    kickerFireCommandFinished = true;

    limitSwitchDepressed = false;

    // command finished, command scheduler will now unschedule it
    cmd.execute();

    // command recognizes that the scheduler has unscheduled the launch command
    cmd.execute();

    limitSwitchDepressed = true;

    // done loading since limit switch depressed
    cmd.execute();

    // load commands finished so command finished
    EXPECT_TRUE(cmd.isFinished());
}

TEST_F(
    HeroAgitatorCommandTest,
    execute_ready_to_fire_refserial_online_firing_stops_when_ref_serial_detected_shot)
{
    bool limitSwitchDepressed = true;
    ON_CALL(drivers.turretMCBCanComm, getLimitSwitchDepressed)
        .WillByDefault(ReturnPointee(&limitSwitchDepressed));

    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(true));
    robotData.turret.heatLimit42 = 100;
    robotData.turret.heat42 = 0;

    ON_CALL(frictionWheels, getDesiredLaunchSpeed).WillByDefault(Return(20));
    ON_CALL(kicker, isOnline).WillByDefault(Return(true));
    ON_CALL(waterwheel, isOnline).WillByDefault(Return(true));

    ON_CALL(kickerFireCommand, isReady).WillByDefault(Return(true));

    ON_CALL(waterwheelLoadCommand, isFinished).WillByDefault(Return(true));
    ON_CALL(kickerLoadCommand, isFinished).WillByDefault(Return(true));

    ON_CALL(kickerFireCommand, isFinished).WillByDefault(Return(false));

    EXPECT_CALL(kickerFireCommand, initialize);

    // schedule the launch command
    cmd.initialize();

    // in the launch state
    cmd.execute();

    // heat goes up, indicating projectile has been launched, loading begins
    robotData.turret.heat42 = 100;
    cmd.execute();

    // loading immediately finishes next execute
    cmd.execute();

    EXPECT_TRUE(cmd.isFinished());
}
