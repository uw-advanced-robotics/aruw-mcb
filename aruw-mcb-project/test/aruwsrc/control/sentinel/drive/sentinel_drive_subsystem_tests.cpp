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

#include "tap/drivers.hpp"

#include "aruwsrc/control/sentinel/drive/sentinel_drive_subsystem.hpp"

using namespace aruwsrc::control::sentinel::drive;
using namespace tap;
using namespace tap::gpio;
using namespace testing;

static constexpr Digital::InputPin LEFT_LIMIT_SWITCH = Digital::A;
static constexpr Digital::InputPin RIGHT_LIMIT_SWITCH = Digital::B;

static constexpr float ENC_RES = tap::motor::DjiMotor::ENC_RESOLUTION;
static constexpr float GEAR_RATIO = SentinelDriveSubsystem::GEAR_RATIO;
static constexpr float WHEEL_RADIUS = SentinelDriveSubsystem::WHEEL_RADIUS;

#define SET_SHAFT_RPM(shaftRpm)                                                     \
    ON_CALL(sentinelDrive.rightWheel, getShaftRPM).WillByDefault(Return(shaftRpm)); \
    ON_CALL(sentinelDrive.leftWheel, getShaftRPM).WillByDefault(Return(shaftRpm));

#define SET_EXPECTED_OUTPUT(output)                                  \
    EXPECT_CALL(sentinelDrive.rightWheel, setDesiredOutput(output)); \
    EXPECT_CALL(sentinelDrive.leftWheel, setDesiredOutput(output));

static constexpr float calculatePosition(int encTicks)
{
    return (static_cast<float>(encTicks) / ENC_RES) * 2.0f * M_PI * WHEEL_RADIUS / GEAR_RATIO;
}

TEST(SentinelDriveSubsystem, initialize_initializes_both_motors_and_configs_pins)
{
    Drivers drivers;
    SentinelDriveSubsystem sentinelDrive(&drivers, LEFT_LIMIT_SWITCH, RIGHT_LIMIT_SWITCH);

    EXPECT_CALL(sentinelDrive.leftWheel, initialize);
    EXPECT_CALL(sentinelDrive.rightWheel, initialize);
    EXPECT_CALL(drivers.digital, configureInputPullMode(LEFT_LIMIT_SWITCH, _));
    EXPECT_CALL(drivers.digital, configureInputPullMode(RIGHT_LIMIT_SWITCH, _));

    sentinelDrive.initialize();
}

TEST(SentinelDriveSubsystem, initialize_with_same_input_pins_raises_error)
{
    Drivers drivers;
    SentinelDriveSubsystem sentinelDrive(&drivers, LEFT_LIMIT_SWITCH, LEFT_LIMIT_SWITCH);

    EXPECT_CALL(drivers.errorController, addToErrorList);

    sentinelDrive.initialize();
}

TEST(SentinelDriveSubsystem, absolutePosition_returns_0_when_motors_offline)
{
    Drivers drivers;
    SentinelDriveSubsystem sentinelDrive(&drivers, LEFT_LIMIT_SWITCH, RIGHT_LIMIT_SWITCH);

    EXPECT_CALL(drivers.errorController, addToErrorList);
    ON_CALL(sentinelDrive.leftWheel, isMotorOnline).WillByDefault(Return(false));
    ON_CALL(sentinelDrive.rightWheel, isMotorOnline).WillByDefault(Return(false));

    EXPECT_FLOAT_EQ(0.0f, sentinelDrive.absolutePosition());
}

TEST(SentinelDriveSubsystem, absolutePosition_returns_right_pos_when_left_not_connected)
{
    static constexpr int ENC_TICKS = 1000;
    Drivers drivers;
    SentinelDriveSubsystem sentinelDrive(&drivers, LEFT_LIMIT_SWITCH, RIGHT_LIMIT_SWITCH);

    EXPECT_CALL(drivers.errorController, addToErrorList);
    ON_CALL(sentinelDrive.leftWheel, isMotorOnline).WillByDefault(Return(false));
    ON_CALL(sentinelDrive.rightWheel, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(sentinelDrive.rightWheel, getEncoderUnwrapped).WillByDefault(Return(ENC_TICKS));

    EXPECT_FLOAT_EQ(calculatePosition(ENC_TICKS), sentinelDrive.absolutePosition());
}

TEST(SentinelDriveSubsystem, absolutePosition_returns_left_pos_when_right_not_connected)
{
    static constexpr int ENC_TICKS = 1000;
    Drivers drivers;
    SentinelDriveSubsystem sentinelDrive(&drivers, LEFT_LIMIT_SWITCH, RIGHT_LIMIT_SWITCH);

    EXPECT_CALL(drivers.errorController, addToErrorList);
    ON_CALL(sentinelDrive.rightWheel, isMotorOnline).WillByDefault(Return(false));
    ON_CALL(sentinelDrive.leftWheel, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(sentinelDrive.leftWheel, getEncoderUnwrapped).WillByDefault(Return(ENC_TICKS));

    EXPECT_FLOAT_EQ(calculatePosition(ENC_TICKS), sentinelDrive.absolutePosition());
}

TEST(SentinelDriveSubsystem, absolutePosition_returns_wheel_avg_when_both_connected)
{
    Drivers drivers;
    SentinelDriveSubsystem sentinelDrive(&drivers, LEFT_LIMIT_SWITCH, RIGHT_LIMIT_SWITCH);

    ON_CALL(sentinelDrive.rightWheel, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(sentinelDrive.leftWheel, isMotorOnline).WillByDefault(Return(true));

    ON_CALL(sentinelDrive.leftWheel, getEncoderUnwrapped).WillByDefault(Return(2000));
    ON_CALL(sentinelDrive.rightWheel, getEncoderUnwrapped).WillByDefault(Return(100));
    EXPECT_FLOAT_EQ(calculatePosition((2000 + 100) / 2), sentinelDrive.absolutePosition());

    ON_CALL(sentinelDrive.leftWheel, getEncoderUnwrapped).WillByDefault(Return(2000));
    ON_CALL(sentinelDrive.rightWheel, getEncoderUnwrapped).WillByDefault(Return(-2000));
    EXPECT_FLOAT_EQ(calculatePosition(0), sentinelDrive.absolutePosition());

    ON_CALL(sentinelDrive.leftWheel, getEncoderUnwrapped).WillByDefault(Return(20000));
    ON_CALL(sentinelDrive.rightWheel, getEncoderUnwrapped).WillByDefault(Return(20000));
    EXPECT_FLOAT_EQ(calculatePosition(20000), sentinelDrive.absolutePosition());
}

TEST(SentinelDriveSubsystem, desired_output_reasonable_for_various_setpoints)
{
    Drivers drivers;
    SentinelDriveSubsystem sentinelDrive(&drivers, LEFT_LIMIT_SWITCH, RIGHT_LIMIT_SWITCH);

    ON_CALL(sentinelDrive.rightWheel, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(sentinelDrive.leftWheel, isMotorOnline).WillByDefault(Return(true));

    // 0 desired and actual = 0 output
    sentinelDrive.setDesiredRpm(0);
    SET_SHAFT_RPM(0);
    SET_EXPECTED_OUTPUT(0);
    sentinelDrive.refresh();

    // 1000 desired, 0 actual = positive output
    sentinelDrive.setDesiredRpm(1000);
    SET_SHAFT_RPM(0);
    SET_EXPECTED_OUTPUT(Gt(0));
    sentinelDrive.refresh();

    // 0 desired, 1000 actual = negative output
    sentinelDrive.setDesiredRpm(0);
    SET_SHAFT_RPM(1000);
    SET_EXPECTED_OUTPUT(Lt(0));
    sentinelDrive.refresh();

    // -1000 desired, 0 actual = negative output
    sentinelDrive.setDesiredRpm(-1000);
    SET_SHAFT_RPM(0);
    SET_EXPECTED_OUTPUT(Lt(0));
    sentinelDrive.refresh();

    // 0 desired, -1000 actual = positive output
    sentinelDrive.setDesiredRpm(0);
    SET_SHAFT_RPM(-1000);
    SET_EXPECTED_OUTPUT(Gt(0));
    sentinelDrive.refresh();
}

TEST(SentinelDriveSubsystem, runHardwareTests_sets_complete_if_shaftRPM_large)
{
    Drivers drivers;
    SentinelDriveSubsystem sentinelDrive(&drivers, LEFT_LIMIT_SWITCH, RIGHT_LIMIT_SWITCH);

    // large negative, tests not complete
    ON_CALL(sentinelDrive.leftWheel, getShaftRPM).WillByDefault(Return(-1000));
    ON_CALL(sentinelDrive.rightWheel, getShaftRPM).WillByDefault(Return(-1000));

    sentinelDrive.runHardwareTests();

    EXPECT_FALSE(sentinelDrive.isHardwareTestComplete());

    // small positive, tests not complete
    ON_CALL(sentinelDrive.leftWheel, getShaftRPM).WillByDefault(Return(10));
    ON_CALL(sentinelDrive.rightWheel, getShaftRPM).WillByDefault(Return(10));

    sentinelDrive.runHardwareTests();

    EXPECT_FALSE(sentinelDrive.isHardwareTestComplete());

    // one small positive, tests not complete
    ON_CALL(sentinelDrive.leftWheel, getShaftRPM).WillByDefault(Return(10));
    ON_CALL(sentinelDrive.rightWheel, getShaftRPM).WillByDefault(Return(10000));

    sentinelDrive.runHardwareTests();

    EXPECT_FALSE(sentinelDrive.isHardwareTestComplete());

    // two large positive, tests complete
    ON_CALL(sentinelDrive.leftWheel, getShaftRPM).WillByDefault(Return(10000));
    ON_CALL(sentinelDrive.rightWheel, getShaftRPM).WillByDefault(Return(10000));

    sentinelDrive.runHardwareTests();

    EXPECT_TRUE(sentinelDrive.isHardwareTestComplete());
}
