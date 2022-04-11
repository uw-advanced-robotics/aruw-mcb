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

#include "aruwsrc/control/sentinel/drive/sentinel_drive_subsystem.hpp"
#include "aruwsrc/drivers.hpp"

using namespace aruwsrc::control::sentinel::drive;
using namespace tap::gpio;
using namespace testing;

static constexpr Digital::InputPin LEFT_LIMIT_SWITCH = Digital::A;
static constexpr Digital::InputPin RIGHT_LIMIT_SWITCH = Digital::B;

static constexpr float ENC_RES = tap::motor::DjiMotor::ENC_RESOLUTION;
static constexpr float GEAR_RATIO = SentinelDriveSubsystem::GEAR_RATIO;
static constexpr float WHEEL_RADIUS = SentinelDriveSubsystem::WHEEL_RADIUS;

static constexpr float calculatePosition(int encTicks)
{
    return (static_cast<float>(encTicks) / ENC_RES) * 2.0f * M_PI * WHEEL_RADIUS / GEAR_RATIO;
}

class SentinelDriveSubsystemTest : public Test
{
protected:
    SentinelDriveSubsystemTest() : sentinelDrive(&drivers, LEFT_LIMIT_SWITCH, RIGHT_LIMIT_SWITCH) {}

    aruwsrc::Drivers drivers;
    SentinelDriveSubsystem sentinelDrive;
};

TEST_F(SentinelDriveSubsystemTest, initialize_initializes_both_motors_and_configs_pins)
{
    EXPECT_CALL(sentinelDrive.leftWheel, initialize);
    EXPECT_CALL(sentinelDrive.rightWheel, initialize);
    EXPECT_CALL(drivers.digital, configureInputPullMode(LEFT_LIMIT_SWITCH, _));
    EXPECT_CALL(drivers.digital, configureInputPullMode(RIGHT_LIMIT_SWITCH, _));

    sentinelDrive.initialize();
}

TEST_F(SentinelDriveSubsystemTest, initialize_with_same_input_pins_raises_error)
{
    aruwsrc::Drivers drivers;
    SentinelDriveSubsystem sentinelDrive(&drivers, LEFT_LIMIT_SWITCH, LEFT_LIMIT_SWITCH);

    EXPECT_CALL(drivers.errorController, addToErrorList);

    sentinelDrive.initialize();
}

TEST_F(SentinelDriveSubsystemTest, absolutePosition_returns_0_when_motors_offline)
{
    EXPECT_CALL(drivers.errorController, addToErrorList);
    ON_CALL(sentinelDrive.leftWheel, isMotorOnline).WillByDefault(Return(false));
    ON_CALL(sentinelDrive.rightWheel, isMotorOnline).WillByDefault(Return(false));

    EXPECT_NEAR(0.0f, sentinelDrive.absolutePosition(), 1E-3);
}

TEST_F(SentinelDriveSubsystemTest, absolutePosition_returns_right_pos_when_left_not_connected)
{
    static constexpr int ENC_TICKS = 1000;

    EXPECT_CALL(drivers.errorController, addToErrorList);
    ON_CALL(sentinelDrive.leftWheel, isMotorOnline).WillByDefault(Return(false));
    ON_CALL(sentinelDrive.rightWheel, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(sentinelDrive.rightWheel, getEncoderUnwrapped).WillByDefault(Return(ENC_TICKS));

    EXPECT_NEAR(calculatePosition(ENC_TICKS), sentinelDrive.absolutePosition(), 1E-3);
}

TEST_F(SentinelDriveSubsystemTest, absolutePosition_returns_left_pos_when_right_not_connected)
{
    static constexpr int ENC_TICKS = 1000;

    EXPECT_CALL(drivers.errorController, addToErrorList);
    ON_CALL(sentinelDrive.rightWheel, isMotorOnline).WillByDefault(Return(false));
    ON_CALL(sentinelDrive.leftWheel, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(sentinelDrive.leftWheel, getEncoderUnwrapped).WillByDefault(Return(ENC_TICKS));

    EXPECT_NEAR(calculatePosition(ENC_TICKS), sentinelDrive.absolutePosition(), 1E-3);
}

TEST_F(SentinelDriveSubsystemTest, absolutePosition_returns_wheel_avg_when_both_connected)
{
    ON_CALL(sentinelDrive.rightWheel, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(sentinelDrive.leftWheel, isMotorOnline).WillByDefault(Return(true));

    int32_t leftEncUnwrapped = 0;
    int32_t rightEncUnwrapped = 0;

    ON_CALL(sentinelDrive.leftWheel, getEncoderUnwrapped)
        .WillByDefault(ReturnPointee(&leftEncUnwrapped));
    ON_CALL(sentinelDrive.rightWheel, getEncoderUnwrapped)
        .WillByDefault(ReturnPointee(&rightEncUnwrapped));

    leftEncUnwrapped = 2000;
    rightEncUnwrapped = 100;
    EXPECT_NEAR(calculatePosition((2'000 + 100) / 2), sentinelDrive.absolutePosition(), 1E-3);

    leftEncUnwrapped = 2'000;
    rightEncUnwrapped = -2'000;
    EXPECT_NEAR(calculatePosition(0), sentinelDrive.absolutePosition(), 1E-3);

    leftEncUnwrapped = 20'000;
    rightEncUnwrapped = 20'000;
    EXPECT_NEAR(calculatePosition(20'000), sentinelDrive.absolutePosition(), 1E-3);
}

TEST_F(SentinelDriveSubsystemTest, desired_output_reasonable_for_various_setpoints)
{
    ON_CALL(sentinelDrive.rightWheel, isMotorOnline).WillByDefault(Return(true));
    ON_CALL(sentinelDrive.leftWheel, isMotorOnline).WillByDefault(Return(true));

    int16_t shaftRpm = 0;

    ON_CALL(sentinelDrive.rightWheel, getShaftRPM).WillByDefault(ReturnPointee(&shaftRpm));
    ON_CALL(sentinelDrive.leftWheel, getShaftRPM).WillByDefault(ReturnPointee(&shaftRpm));

    {
        InSequence sequence;
        EXPECT_CALL(sentinelDrive.rightWheel, setDesiredOutput(0));
        EXPECT_CALL(sentinelDrive.rightWheel, setDesiredOutput(Gt(0)));
        EXPECT_CALL(sentinelDrive.rightWheel, setDesiredOutput(Lt(0)));
        EXPECT_CALL(sentinelDrive.rightWheel, setDesiredOutput(Lt(0)));
        EXPECT_CALL(sentinelDrive.rightWheel, setDesiredOutput(Gt(0)));
    }

    {
        InSequence sequence;
        EXPECT_CALL(sentinelDrive.leftWheel, setDesiredOutput(0));
        EXPECT_CALL(sentinelDrive.leftWheel, setDesiredOutput(Gt(0)));
        EXPECT_CALL(sentinelDrive.leftWheel, setDesiredOutput(Lt(0)));
        EXPECT_CALL(sentinelDrive.leftWheel, setDesiredOutput(Lt(0)));
        EXPECT_CALL(sentinelDrive.leftWheel, setDesiredOutput(Gt(0)));
    }

    // 0 desired and actual = 0 output
    sentinelDrive.setDesiredRpm(0);
    shaftRpm = 0;
    sentinelDrive.refresh();

    // 1000 desired, 0 actual = positive output
    sentinelDrive.setDesiredRpm(1000);
    shaftRpm = 0;
    sentinelDrive.refresh();

    // 0 desired, 1000 actual = negative output
    sentinelDrive.setDesiredRpm(0);
    shaftRpm = 1000;
    sentinelDrive.refresh();

    // -1000 desired, 0 actual = negative output
    sentinelDrive.setDesiredRpm(-1000);
    shaftRpm = 0;
    sentinelDrive.refresh();

    // 0 desired, -1000 actual = positive output
    sentinelDrive.setDesiredRpm(0);
    shaftRpm = -1000;
    sentinelDrive.refresh();
}

TEST_F(SentinelDriveSubsystemTest, runHardwareTests_sets_complete_if_shaftRPM_large)
{
    int16_t leftShaftRpm = 0;
    int16_t rightShaftRpm = 0;

    ON_CALL(sentinelDrive.rightWheel, getShaftRPM).WillByDefault(ReturnPointee(&leftShaftRpm));
    ON_CALL(sentinelDrive.leftWheel, getShaftRPM).WillByDefault(ReturnPointee(&rightShaftRpm));

    // large negative, tests not complete
    leftShaftRpm = -1'000;
    rightShaftRpm = -1'000;
    sentinelDrive.runHardwareTests();

    EXPECT_FALSE(sentinelDrive.isHardwareTestComplete());

    // small positive, tests not complete
    leftShaftRpm = 10;
    rightShaftRpm = 10;
    sentinelDrive.runHardwareTests();

    EXPECT_FALSE(sentinelDrive.isHardwareTestComplete());

    // one small positive, tests not complete
    leftShaftRpm = 10;
    rightShaftRpm = 10'000;
    sentinelDrive.runHardwareTests();

    EXPECT_FALSE(sentinelDrive.isHardwareTestComplete());

    // two large positive, tests complete
    leftShaftRpm = 10'000;
    rightShaftRpm = 10'000;
    sentinelDrive.runHardwareTests();

    EXPECT_TRUE(sentinelDrive.isHardwareTestComplete());
}
