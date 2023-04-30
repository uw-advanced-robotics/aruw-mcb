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

#include "aruwsrc/control/chassis/sentry/sentry_drive_evade_command.hpp"
#include "aruwsrc/mock/sentry_drive_subsystem_mock.hpp"

using namespace testing;
using namespace aruwsrc::control::sentry::drive;

class SentryDriveEvadeCommandTest : public Test
{
protected:
    SentryDriveEvadeCommandTest()
        : sub(&drivers, tap::gpio::Digital::InputPin::B, tap::gpio::Digital::InputPin::C),
          cmd(&sub, 1)
    {
    }

    void SetUp() override
    {
        ON_CALL(sub, setDesiredRpm)
            .WillByDefault([&](float desiredRpm) { this->desiredRpm = desiredRpm; });

        ON_CALL(sub, getDesiredRpm).WillByDefault(ReturnPointee(&desiredRpm));

        ON_CALL(sub, getAbsolutePosition).WillByDefault(ReturnPointee(&position));
    }

private:
    tap::Drivers drivers;

protected:
    NiceMock<aruwsrc::mock::SentryDriveSubsystemMock> sub;
    SentryDriveEvadeCommand cmd;
    float position = 0;
    float desiredRpm = 0;
};

TEST_F(SentryDriveEvadeCommandTest, initiailze_sets_new_desired_rpm)
{
    EXPECT_CALL(sub, setDesiredRpm(Not(0)));
    cmd.initialize();
}

TEST_F(SentryDriveEvadeCommandTest, initialize_sets_positive_desired_rpm_if_currently_negative_rpm)
{
    ON_CALL(sub, getDesiredRpm).WillByDefault(Return(-SentryDriveEvadeCommand::MIN_RPM));
    EXPECT_CALL(sub, setDesiredRpm(SentryDriveEvadeCommand::MIN_RPM));
    cmd.initialize();
}

TEST_F(SentryDriveEvadeCommandTest, initialize_sets_negative_desired_rpm_if_currently_positive_rpm)
{
    ON_CALL(sub, getDesiredRpm).WillByDefault(Return(SentryDriveEvadeCommand::MIN_RPM));
    EXPECT_CALL(sub, setDesiredRpm(-SentryDriveEvadeCommand::MIN_RPM));
    cmd.initialize();
}

TEST_F(SentryDriveEvadeCommandTest, end_sets_desired_rpm_to_0)
{
    EXPECT_CALL(sub, setDesiredRpm(FloatNear(0, 1E-5))).Times(2);
    cmd.end(true);
    cmd.end(false);
}

TEST_F(SentryDriveEvadeCommandTest, execute_direction_changes_immediately_when_at_start)
{
    position = 0;

    cmd.initialize();

    desiredRpm = -1000;

    cmd.execute();

    EXPECT_NEAR(SentryDriveEvadeCommand::MIN_RPM, desiredRpm, 1E-5);
}

TEST_F(SentryDriveEvadeCommandTest, execute_direction_changes_immediately_when_at_end)
{
    position = SentryDriveSubsystem::RAIL_LENGTH - SentryDriveSubsystem::SENTRY_LENGTH;

    cmd.initialize();

    desiredRpm = 1000;

    cmd.execute();

    EXPECT_NEAR(-SentryDriveEvadeCommand::MIN_RPM, desiredRpm, 1E-5);
}

TEST_F(SentryDriveEvadeCommandTest, execute_direction_changes_immediately_when_close_to_start)
{
    position = SentryDriveEvadeCommand::TURNAROUND_BUFFER;

    cmd.initialize();

    desiredRpm = -1000;

    cmd.execute();

    EXPECT_NEAR(SentryDriveEvadeCommand::MIN_RPM, desiredRpm, 1E-5);
}

TEST_F(SentryDriveEvadeCommandTest, execute_direction_changes_immediately_when_close_to_end)
{
    position = SentryDriveSubsystem::RAIL_LENGTH - SentryDriveSubsystem::SENTRY_LENGTH -
               SentryDriveEvadeCommand::TURNAROUND_BUFFER;

    cmd.initialize();

    desiredRpm = 1000;

    cmd.execute();

    EXPECT_NEAR(-SentryDriveEvadeCommand::MIN_RPM, desiredRpm, 1E-5);
}

TEST_F(SentryDriveEvadeCommandTest, execute_direction_changes_after_some_distance_traveled)
{
    position = 1;
    if (SentryDriveEvadeCommand::TURNAROUND_BUFFER >
        SentryDriveEvadeCommand::LARGE_ARMOR_PLATE_WIDTH)
    {
        position += SentryDriveEvadeCommand::TURNAROUND_BUFFER -
                    SentryDriveEvadeCommand::LARGE_ARMOR_PLATE_WIDTH;
    }
    else
    {
        position += SentryDriveEvadeCommand::LARGE_ARMOR_PLATE_WIDTH;
    }

    desiredRpm = -1;
    cmd.initialize();

    EXPECT_NEAR(SentryDriveEvadeCommand::LARGE_ARMOR_PLATE_WIDTH, cmd.getDistanceToDrive(), 1E-5);

    cmd.execute();

    EXPECT_NEAR(SentryDriveEvadeCommand::MIN_RPM, desiredRpm, 1E-5);

    position += SentryDriveEvadeCommand::LARGE_ARMOR_PLATE_WIDTH / 2;

    cmd.execute();

    EXPECT_NEAR(SentryDriveEvadeCommand::MIN_RPM, desiredRpm, 1E-5);

    position += SentryDriveEvadeCommand::LARGE_ARMOR_PLATE_WIDTH / 2;

    cmd.execute();

    EXPECT_NEAR(-SentryDriveEvadeCommand::MIN_RPM, desiredRpm, 1E-5);
}

TEST_F(
    SentryDriveEvadeCommandTest,
    execute_direction_changes_negative_to_positive_after_distance_traveled)
{
    position = SentryDriveSubsystem::RAIL_LENGTH - SentryDriveSubsystem::SENTRY_LENGTH - 1;
    if (SentryDriveEvadeCommand::TURNAROUND_BUFFER >
        SentryDriveEvadeCommand::LARGE_ARMOR_PLATE_WIDTH)
    {
        position -= SentryDriveEvadeCommand::TURNAROUND_BUFFER -
                    SentryDriveEvadeCommand::LARGE_ARMOR_PLATE_WIDTH;
    }
    else
    {
        position -= SentryDriveEvadeCommand::LARGE_ARMOR_PLATE_WIDTH;
    }

    desiredRpm = 1;
    cmd.initialize();

    EXPECT_NEAR(SentryDriveEvadeCommand::LARGE_ARMOR_PLATE_WIDTH, cmd.getDistanceToDrive(), 1E-5);

    cmd.execute();

    EXPECT_NEAR(-SentryDriveEvadeCommand::MIN_RPM, desiredRpm, 1E-5);

    position -= SentryDriveEvadeCommand::LARGE_ARMOR_PLATE_WIDTH / 2;

    cmd.execute();

    EXPECT_NEAR(-SentryDriveEvadeCommand::MIN_RPM, desiredRpm, 1E-5);

    position -= SentryDriveEvadeCommand::LARGE_ARMOR_PLATE_WIDTH / 2;

    cmd.execute();

    EXPECT_NEAR(SentryDriveEvadeCommand::MIN_RPM, desiredRpm, 1E-5);
}

TEST_F(
    SentryDriveEvadeCommandTest,
    execute_distance_to_travel_larger_when_turning_around_inside_turnaround_buffer_near_start_of_rail)
{
    position = 0;
    cmd.initialize();

    desiredRpm = -1000;

    cmd.execute();

    EXPECT_NEAR(
        (SentryDriveSubsystem::RAIL_LENGTH - SentryDriveSubsystem::SENTRY_LENGTH) / 2,
        cmd.getDistanceToDrive(),
        1E-5);

    desiredRpm = -1000;
    position = SentryDriveEvadeCommand::TURNAROUND_BUFFER / 2;

    cmd.execute();

    EXPECT_NEAR(
        (SentryDriveSubsystem::RAIL_LENGTH - SentryDriveSubsystem::SENTRY_LENGTH) / 2 - position,
        cmd.getDistanceToDrive(),
        1E-5);
}

TEST_F(
    SentryDriveEvadeCommandTest,
    execute_distance_to_travel_larger_when_turning_around_inside_turnaround_buffer_near_end_of_rail)
{
    position = SentryDriveSubsystem::RAIL_LENGTH - SentryDriveSubsystem::SENTRY_LENGTH;
    cmd.initialize();

    desiredRpm = 1000;

    cmd.execute();

    EXPECT_NEAR(
        (SentryDriveSubsystem::RAIL_LENGTH - SentryDriveSubsystem::SENTRY_LENGTH) / 2,
        cmd.getDistanceToDrive(),
        1E-5);

    desiredRpm = 1000;
    position -= SentryDriveEvadeCommand::TURNAROUND_BUFFER / 2;

    cmd.execute();

    EXPECT_NEAR(
        (SentryDriveSubsystem::RAIL_LENGTH - SentryDriveSubsystem::SENTRY_LENGTH) / 2 -
            SentryDriveEvadeCommand::TURNAROUND_BUFFER / 2,
        cmd.getDistanceToDrive(),
        1E-5);
}
