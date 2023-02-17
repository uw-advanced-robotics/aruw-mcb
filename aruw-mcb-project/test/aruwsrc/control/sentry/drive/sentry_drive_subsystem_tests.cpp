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

#include "aruwsrc/robot/sentry/drive/sentry_drive_subsystem.hpp"

using namespace aruwsrc::control::sentry::drive;
using namespace tap::gpio;
using namespace testing;

static constexpr Digital::InputPin LEFT_LIMIT_SWITCH = Digital::B;
static constexpr Digital::InputPin RIGHT_LIMIT_SWITCH = Digital::C;

static constexpr float ENC_RES = tap::motor::DjiMotor::ENC_RESOLUTION;
static constexpr float GEAR_RATIO = SentryDriveSubsystem::GEAR_RATIO;
static constexpr float WHEEL_RADIUS = SentryDriveSubsystem::WHEEL_RADIUS;

static constexpr float calculatePosition(int encTicks)
{
    return (static_cast<float>(encTicks) / ENC_RES) * 2.0f * M_PI * WHEEL_RADIUS / GEAR_RATIO;
}

class SentryDriveSubsystemTest : public Test
{
protected:
    SentryDriveSubsystemTest() : sentryDrive(&drivers, LEFT_LIMIT_SWITCH, RIGHT_LIMIT_SWITCH) {}

    tap::Drivers drivers;
    SentryDriveSubsystem sentryDrive;
};

TEST_F(SentryDriveSubsystemTest, initialize_initializes_both_motors_and_configs_pins)
{
    EXPECT_CALL(sentryDrive.leftWheel, initialize);
    EXPECT_CALL(drivers.digital, configureInputPullMode(LEFT_LIMIT_SWITCH, _));
    EXPECT_CALL(drivers.digital, configureInputPullMode(RIGHT_LIMIT_SWITCH, _));

    sentryDrive.initialize();
}

TEST_F(SentryDriveSubsystemTest, initialize_with_same_input_pins_raises_error)
{
    tap::Drivers drivers;
    SentryDriveSubsystem sentryDrive(&drivers, LEFT_LIMIT_SWITCH, LEFT_LIMIT_SWITCH);

    EXPECT_CALL(drivers.errorController, addToErrorList);

    sentryDrive.initialize();
}

TEST_F(SentryDriveSubsystemTest, getAbsolutePosition_returns_0_when_motors_offline)
{
    EXPECT_CALL(drivers.errorController, addToErrorList);
    ON_CALL(sentryDrive.leftWheel, isMotorOnline).WillByDefault(Return(false));

    EXPECT_NEAR(0.0f, sentryDrive.getAbsolutePosition(), 1E-3);
}

TEST_F(SentryDriveSubsystemTest, desired_output_reasonable_for_various_setpoints)
{
    ON_CALL(sentryDrive.leftWheel, isMotorOnline).WillByDefault(Return(true));

    int16_t shaftRpm = 0;

    ON_CALL(sentryDrive.leftWheel, getShaftRPM).WillByDefault(ReturnPointee(&shaftRpm));

    {
        InSequence sequence;
        EXPECT_CALL(sentryDrive.leftWheel, setDesiredOutput(0));
        EXPECT_CALL(sentryDrive.leftWheel, setDesiredOutput(Gt(0)));
        EXPECT_CALL(sentryDrive.leftWheel, setDesiredOutput(Lt(0)));
        EXPECT_CALL(sentryDrive.leftWheel, setDesiredOutput(Lt(0)));
        EXPECT_CALL(sentryDrive.leftWheel, setDesiredOutput(Gt(0)));
    }

    // 0 desired and actual = 0 output
    sentryDrive.setDesiredRpm(0);
    shaftRpm = 0;
    sentryDrive.refresh();

    // 1000 desired, 0 actual = positive output
    sentryDrive.setDesiredRpm(1000);
    shaftRpm = 0;
    sentryDrive.refresh();

    // 0 desired, 1000 actual = negative output
    sentryDrive.setDesiredRpm(0);
    shaftRpm = 1000;
    sentryDrive.refresh();

    // -1000 desired, 0 actual = negative output
    sentryDrive.setDesiredRpm(-1000);
    shaftRpm = 0;
    sentryDrive.refresh();

    // 0 desired, -1000 actual = positive output
    sentryDrive.setDesiredRpm(0);
    shaftRpm = -1000;
    sentryDrive.refresh();
}
