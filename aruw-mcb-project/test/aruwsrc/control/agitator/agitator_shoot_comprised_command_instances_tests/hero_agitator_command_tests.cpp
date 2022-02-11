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

#include "aruwsrc/control/agitator/hero_agitator_command.hpp"
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/mock/agitator_subsystem_mock.hpp"
#include "aruwsrc/mock/friction_wheel_subsystem_mock.hpp"

using namespace aruwsrc;
using namespace aruwsrc::agitator;
using namespace aruwsrc::mock;
using namespace tap::communication::serial;
using namespace testing;

using tap::arch::clock::setTime;

#define SETUP_TEST(heroAgitatorCommandConfig)                      \
    Drivers drivers;                                               \
    NiceMock<AgitatorSubsystemMock> kicker(&drivers);              \
    NiceMock<AgitatorSubsystemMock> waterwheel(&drivers);          \
    NiceMock<FrictionWheelSubsystemMock> frictionWheels(&drivers); \
    HeroAgitatorCommand cmd(                                       \
        &drivers,                                                  \
        &kicker,                                                   \
        &waterwheel,                                               \
        &frictionWheels,                                           \
        heroAgitatorCommandConfig);

static HeroAgitatorCommand::Config DEFAULT_HERO_AGITATOR_CMD_CONFIG{
    .kickerShootRotateAngle = M_PI_2,
    .kickerShootRotateTime = 10,
    .kickerShootSetpointTolerance = M_PI,
    .kickerLoadRotateAngle = M_PI,
    .kickerLoadSetpointTolerance = M_PI / 32,
    .waterwheelLoadRotateAngle = M_PI_4,
    .waterwheelLoadSetpointTolerance = M_PI / 32,
    .loadRotateTime = 20,
    .waterwheelUnjamDisplacement = M_PI_4,
    .waterwheelUnjamThreshold = M_PI / 14,
    .waterwheelUnjamMaxWaitTime = 20,
    .heatLimiting = false,
    .heatLimitBuffer = 100,
};

TEST(HeroAgitatorCommand, isReady_no_heat_limiting_true_when_both_motors_online)
{
    SETUP_TEST(DEFAULT_HERO_AGITATOR_CMD_CONFIG);

    RefSerial::Rx::RobotData robotData;
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));
    ON_CALL(frictionWheels, getDesiredLaunchSpeed).WillByDefault(Return(20));
    ON_CALL(kicker, isOnline).WillByDefault(Return(true));
    ON_CALL(waterwheel, isOnline).WillByDefault(Return(true));

    EXPECT_TRUE(cmd.isReady());
}

TEST(HeroAgitatorCommand, isReady_no_heat_limiting_false_when_single_motor_offline)
{
    SETUP_TEST(DEFAULT_HERO_AGITATOR_CMD_CONFIG);

    RefSerial::Rx::RobotData robotData;
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));
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

TEST(HeroAgitatorCommand, isReady_no_heat_limiting_true_when_heat_limit_within_buffer)
{
    SETUP_TEST(DEFAULT_HERO_AGITATOR_CMD_CONFIG);

    RefSerial::Rx::RobotData robotData;
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));
    ON_CALL(frictionWheels, getDesiredLaunchSpeed).WillByDefault(Return(20));
    ON_CALL(kicker, isOnline).WillByDefault(Return(true));
    ON_CALL(waterwheel, isOnline).WillByDefault(Return(true));

    robotData.turret.heatLimit42 = 200;
    robotData.turret.heat42 = 150;

    EXPECT_TRUE(cmd.isReady());
}

TEST(HeroAgitatorCommand, isReady_heat_limiting_true_when_heat_limit_below_buffer)
{
    auto agitatorConfig = DEFAULT_HERO_AGITATOR_CMD_CONFIG;
    agitatorConfig.heatLimiting = true;
    SETUP_TEST(agitatorConfig);

    RefSerial::Rx::RobotData robotData;
    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(true));
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));
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

TEST(HeroAgitatorCommand, isReady_heat_limiting_false_when_heat_limit_above_buffer)
{
    auto agitatorConfig = DEFAULT_HERO_AGITATOR_CMD_CONFIG;
    agitatorConfig.heatLimiting = true;
    SETUP_TEST(agitatorConfig);

    RefSerial::Rx::RobotData robotData;
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));
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

TEST(HeroAgitatorCommand, isReady_false_when_flywheels_not_on)
{
    SETUP_TEST(DEFAULT_HERO_AGITATOR_CMD_CONFIG);

    RefSerial::Rx::RobotData robotData;
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));
    ON_CALL(frictionWheels, getDesiredLaunchSpeed).WillByDefault(Return(0));
    ON_CALL(kicker, isOnline).WillByDefault(Return(true));
    ON_CALL(waterwheel, isOnline).WillByDefault(Return(true));

    EXPECT_FALSE(cmd.isReady());
}

TEST(HeroAgitatorCommand, isFinished_true_when_flywheels_not_on)
{
    SETUP_TEST(DEFAULT_HERO_AGITATOR_CMD_CONFIG);

    ON_CALL(frictionWheels, getDesiredLaunchSpeed).WillByDefault(Return(0));
    ON_CALL(kicker, isOnline).WillByDefault(Return(true));
    ON_CALL(waterwheel, isOnline).WillByDefault(Return(true));

    EXPECT_TRUE(cmd.isFinished());
}

TEST(HeroAgitatorCommand, isFinished_true_when_motors_disconnected)
{
    SETUP_TEST(DEFAULT_HERO_AGITATOR_CMD_CONFIG);

    ON_CALL(frictionWheels, getDesiredLaunchSpeed).WillByDefault(Return(20));
    EXPECT_CALL(kicker, isOnline)
        .Times(AnyNumber())
        .WillOnce(Return(false))
        .WillOnce(Return(true))
        .WillOnce(Return(false));
    EXPECT_CALL(waterwheel, isOnline)
        .Times(AnyNumber())
        .WillOnce(Return(true))
        .WillOnce(Return(false))
        .WillOnce(Return(false));

    EXPECT_TRUE(cmd.isFinished());
    EXPECT_TRUE(cmd.isFinished());
    EXPECT_TRUE(cmd.isFinished());
}

TEST(HeroAgitatorCommand, execute_ball_not_loaded_loading_happens)
{
    setTime(0);

    SETUP_TEST(DEFAULT_HERO_AGITATOR_CMD_CONFIG);

    // The first DEFAULT_HERO_AGITATOR_CMD_CONFIG.loadRotateTime getLimitSwitchDepressed returns
    // false, then the last time it returns true
    InSequence seq;
    EXPECT_CALL(drivers.turretMCBCanComm, getLimitSwitchDepressed)
        .Times(DEFAULT_HERO_AGITATOR_CMD_CONFIG.loadRotateTime)
        .WillRepeatedly(Return(false));
    EXPECT_CALL(drivers.turretMCBCanComm, getLimitSwitchDepressed).Times(1).WillOnce(Return(true));

    ON_CALL(frictionWheels, getDesiredLaunchSpeed).WillByDefault(Return(20));
    ON_CALL(kicker, isOnline).WillByDefault(Return(true));
    ON_CALL(waterwheel, isOnline).WillByDefault(Return(true));

    float kickerPosition = 0.0f;
    ON_CALL(kicker, getCurrentValue).WillByDefault(ReturnPointee(&kickerPosition));
    ON_CALL(kicker, setSetpoint).WillByDefault([&](float val) { kickerPosition = val; });
    ON_CALL(kicker, getSetpoint).WillByDefault(ReturnPointee(&kickerPosition));
    float waterwheelPosition = 0.0f;
    ON_CALL(waterwheel, getCurrentValue).WillByDefault(ReturnPointee(&waterwheelPosition));
    ON_CALL(waterwheel, setSetpoint).WillByDefault([&](float val) { waterwheelPosition = val; });
    ON_CALL(waterwheel, getSetpoint).WillByDefault(ReturnPointee(&waterwheelPosition));

    cmd.initialize();

    for (uint16_t i = 0; i < DEFAULT_HERO_AGITATOR_CMD_CONFIG.loadRotateTime; i++)
    {
        float prevKickerPosition = kickerPosition;
        float prevWaterwheelPosition = waterwheelPosition;
        EXPECT_FALSE(cmd.isFinished());
        setTime(i + 1);
        cmd.execute();
        // both positions should be strictly increaseing
        EXPECT_LE(prevKickerPosition, kickerPosition);
        EXPECT_LE(prevWaterwheelPosition, waterwheelPosition);
    }

    // Should be around the load rotate angles but doesn't have to be exact
    EXPECT_NEAR(DEFAULT_HERO_AGITATOR_CMD_CONFIG.kickerLoadRotateAngle, kickerPosition, 1);
    EXPECT_NEAR(DEFAULT_HERO_AGITATOR_CMD_CONFIG.waterwheelLoadRotateAngle, waterwheelPosition, 1);

    EXPECT_TRUE(cmd.isFinished());
}

TEST(HeroAgitatorCommand, execute_ball_not_loaded_multiple_load_cycles_happen)
{
    setTime(0);

    SETUP_TEST(DEFAULT_HERO_AGITATOR_CMD_CONFIG);

    // The first 4 * DEFAULT_HERO_AGITATOR_CMD_CONFIG.loadRotateTime getLimitSwitchDepressed returns
    // false, then the last time it returns true
    InSequence seq;
    ON_CALL(drivers.turretMCBCanComm, getLimitSwitchDepressed).WillByDefault(Return(false));
    EXPECT_CALL(drivers.turretMCBCanComm, getLimitSwitchDepressed)
        .Times(4 * DEFAULT_HERO_AGITATOR_CMD_CONFIG.loadRotateTime)
        .WillRepeatedly(Return(false));
    EXPECT_CALL(drivers.turretMCBCanComm, getLimitSwitchDepressed).Times(1).WillOnce(Return(true));

    ON_CALL(frictionWheels, getDesiredLaunchSpeed).WillByDefault(Return(20));
    ON_CALL(kicker, isOnline).WillByDefault(Return(true));
    ON_CALL(waterwheel, isOnline).WillByDefault(Return(true));

    float kickerPosition = 0.0f;
    ON_CALL(kicker, getCurrentValue).WillByDefault(ReturnPointee(&kickerPosition));
    ON_CALL(kicker, setSetpoint).WillByDefault([&](float val) { kickerPosition = val; });
    ON_CALL(kicker, getSetpoint).WillByDefault(ReturnPointee(&kickerPosition));
    float waterwheelPosition = 0.0f;
    ON_CALL(waterwheel, getCurrentValue).WillByDefault(ReturnPointee(&waterwheelPosition));
    ON_CALL(waterwheel, setSetpoint).WillByDefault([&](float val) { waterwheelPosition = val; });
    ON_CALL(waterwheel, getSetpoint).WillByDefault(ReturnPointee(&waterwheelPosition));

    cmd.initialize();

    for (uint16_t i = 0; i < 4 * DEFAULT_HERO_AGITATOR_CMD_CONFIG.loadRotateTime; i++)
    {
        float prevKickerPosition = kickerPosition;
        float prevWaterwheelPosition = waterwheelPosition;
        EXPECT_FALSE(cmd.isFinished());
        setTime(i + 1);
        cmd.execute();
        // both positions should be increaseing or the same
        EXPECT_LE(prevKickerPosition, kickerPosition);
        EXPECT_LE(prevWaterwheelPosition, waterwheelPosition);
    }

    // Should be around 4 * the load angles, but doesn't have to exact
    EXPECT_NEAR(DEFAULT_HERO_AGITATOR_CMD_CONFIG.kickerLoadRotateAngle * 4, kickerPosition, 1);
    EXPECT_NEAR(
        DEFAULT_HERO_AGITATOR_CMD_CONFIG.waterwheelLoadRotateAngle * 4,
        waterwheelPosition,
        1);

    EXPECT_TRUE(cmd.isFinished());
}

TEST(
    HeroAgitatorCommand,
    execute_ready_to_fire_refserial_offline_firing_happens_then_loading_stops_immediately_when_limit_switch_still_depressed)
{
    setTime(0);

    SETUP_TEST(DEFAULT_HERO_AGITATOR_CMD_CONFIG);

    ON_CALL(drivers.turretMCBCanComm, getLimitSwitchDepressed).WillByDefault(Return(true));

    RefSerial::Rx::RobotData robotData;
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));
    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(false));

    ON_CALL(frictionWheels, getDesiredLaunchSpeed).WillByDefault(Return(20));
    ON_CALL(kicker, isOnline).WillByDefault(Return(true));
    ON_CALL(waterwheel, isOnline).WillByDefault(Return(true));

    float kickerPosition = 0.0f;
    ON_CALL(kicker, getCurrentValue).WillByDefault(ReturnPointee(&kickerPosition));
    ON_CALL(kicker, setSetpoint).WillByDefault([&](float val) { kickerPosition = val; });
    ON_CALL(kicker, getSetpoint).WillByDefault(ReturnPointee(&kickerPosition));
    float waterwheelPosition = 0.0f;
    ON_CALL(waterwheel, getCurrentValue).WillByDefault(ReturnPointee(&waterwheelPosition));
    ON_CALL(waterwheel, setSetpoint).WillByDefault([&](float val) { waterwheelPosition = val; });
    ON_CALL(waterwheel, getSetpoint).WillByDefault(ReturnPointee(&waterwheelPosition));

    cmd.initialize();

    // call execute twice as many times as should be needed to ensure loading state is not entered
    for (uint16_t i = 0; i < 2 * DEFAULT_HERO_AGITATOR_CMD_CONFIG.kickerShootRotateTime; i++)
    {
        float prevKickerPosition = kickerPosition;
        setTime(i + 1);
        cmd.execute();
        EXPECT_LE(prevKickerPosition, kickerPosition);
        EXPECT_NEAR(0.0f, waterwheelPosition, 1E-5);
    }

    EXPECT_NEAR(DEFAULT_HERO_AGITATOR_CMD_CONFIG.kickerShootRotateAngle, kickerPosition, 0.1f);

    EXPECT_TRUE(cmd.isFinished());
}

TEST(
    HeroAgitatorCommand,
    execute_ready_to_fire_refserial_online_firing_stops_when_ref_serial_detected_shot)
{
    setTime(0);

    SETUP_TEST(DEFAULT_HERO_AGITATOR_CMD_CONFIG);

    ON_CALL(drivers.turretMCBCanComm, getLimitSwitchDepressed).WillByDefault(Return(true));

    RefSerial::Rx::RobotData robotData;
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));
    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(true));
    robotData.turret.heatLimit42 = 100;
    robotData.turret.heat42 = 0;

    ON_CALL(frictionWheels, getDesiredLaunchSpeed).WillByDefault(Return(20));
    ON_CALL(kicker, isOnline).WillByDefault(Return(true));
    ON_CALL(waterwheel, isOnline).WillByDefault(Return(true));

    float kickerPosition = 0.0f;
    ON_CALL(kicker, getCurrentValue).WillByDefault(ReturnPointee(&kickerPosition));
    ON_CALL(kicker, setSetpoint).WillByDefault([&](float val) { kickerPosition = val; });
    ON_CALL(kicker, getSetpoint).WillByDefault(ReturnPointee(&kickerPosition));
    float waterwheelPosition = 0.0f;
    ON_CALL(waterwheel, getCurrentValue).WillByDefault(ReturnPointee(&waterwheelPosition));
    ON_CALL(waterwheel, setSetpoint).WillByDefault([&](float val) { waterwheelPosition = val; });
    ON_CALL(waterwheel, getSetpoint).WillByDefault(ReturnPointee(&waterwheelPosition));

    cmd.initialize();

    cmd.execute();
    robotData.turret.heat42 = 100;
    cmd.execute();  // command detects shot fired
    cmd.execute();  // command detects limit switch triggered and finishes

    EXPECT_TRUE(cmd.isFinished());
}

TEST(
    HeroAgitatorCommand,
    execute_ready_to_fire_refserial_online_loading_starts_after_firing_when_limit_switch_not_depressed)
{
    setTime(0);

    SETUP_TEST(DEFAULT_HERO_AGITATOR_CMD_CONFIG);

    EXPECT_CALL(drivers.turretMCBCanComm, getLimitSwitchDepressed)
        .Times(AnyNumber())
        .WillOnce(Return(true))
        .WillRepeatedly(Return(false));

    RefSerial::Rx::RobotData robotData;
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));
    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(true));
    robotData.turret.heatLimit42 = 100;
    robotData.turret.heat42 = 0;

    ON_CALL(frictionWheels, getDesiredLaunchSpeed).WillByDefault(Return(20));
    ON_CALL(kicker, isOnline).WillByDefault(Return(true));
    ON_CALL(waterwheel, isOnline).WillByDefault(Return(true));

    float kickerPosition = 0.0f;
    ON_CALL(kicker, getCurrentValue).WillByDefault(ReturnPointee(&kickerPosition));
    ON_CALL(kicker, setSetpoint).WillByDefault([&](float val) { kickerPosition = val; });
    ON_CALL(kicker, getSetpoint).WillByDefault(ReturnPointee(&kickerPosition));
    float waterwheelPosition = 0.0f;
    ON_CALL(waterwheel, getCurrentValue).WillByDefault(ReturnPointee(&waterwheelPosition));
    ON_CALL(waterwheel, setSetpoint).WillByDefault([&](float val) { waterwheelPosition = val; });
    ON_CALL(waterwheel, getSetpoint).WillByDefault(ReturnPointee(&waterwheelPosition));

    cmd.initialize();

    cmd.execute();
    robotData.turret.heat42 = 100;
    cmd.execute();

    for (uint16_t i = 0; i < 2 * DEFAULT_HERO_AGITATOR_CMD_CONFIG.loadRotateTime; i++)
    {
        float prevKickerPosition = kickerPosition;
        float prevWaterwheelPosition = waterwheelPosition;
        EXPECT_FALSE(cmd.isFinished());
        setTime(i + 1);
        cmd.execute();
        // both positions should be increaseing or the same
        EXPECT_LE(prevKickerPosition, kickerPosition);
        EXPECT_LE(prevWaterwheelPosition, waterwheelPosition);
    }

    EXPECT_NEAR(DEFAULT_HERO_AGITATOR_CMD_CONFIG.kickerLoadRotateAngle * 2, kickerPosition, 1);
    EXPECT_NEAR(
        DEFAULT_HERO_AGITATOR_CMD_CONFIG.waterwheelLoadRotateAngle * 2,
        waterwheelPosition,
        1);
}
