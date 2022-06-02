#include <gtest/gtest.h>

#include "aruwsrc/control/sentinel/drive/sentinel_drive_evade_command.hpp"
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/mock/sentinel_drive_subsystem_mock.hpp"

using namespace testing;
using namespace aruwsrc::control::sentinel::drive;

class SentinelDriveEvadeCommandTest : public Test
{
protected:
    SentinelDriveEvadeCommandTest()
        : sub(&drivers, tap::gpio::Digital::InputPin::B, tap::gpio::Digital::InputPin::C),
          cmd(&sub, 1)
    {
    }

    void SetUp() override
    {
        ON_CALL(sub, setDesiredRpm).WillByDefault([&](float desiredRpm) {
            this->desiredRpm = desiredRpm;
        });

        ON_CALL(sub, getDesiredRpm).WillByDefault(ReturnPointee(&desiredRpm));

        ON_CALL(sub, absolutePosition).WillByDefault(ReturnPointee(&position));
    }

private:
    aruwsrc::Drivers drivers;

protected:
    NiceMock<aruwsrc::mock::SentinelDriveSubsystemMock> sub;
    SentinelDriveEvadeCommand cmd;
    float position = 0;
    float desiredRpm = 0;
};

TEST_F(SentinelDriveEvadeCommandTest, initiailze_sets_new_desired_rpm)
{
    EXPECT_CALL(sub, setDesiredRpm(Not(0)));
    cmd.initialize();
}

TEST_F(
    SentinelDriveEvadeCommandTest,
    initialize_sets_positive_desired_rpm_if_currently_negative_rpm)
{
    ON_CALL(sub, getDesiredRpm).WillByDefault(Return(-SentinelDriveEvadeCommand::MIN_RPM));
    EXPECT_CALL(sub, setDesiredRpm(SentinelDriveEvadeCommand::MIN_RPM));
    cmd.initialize();
}

TEST_F(
    SentinelDriveEvadeCommandTest,
    initialize_sets_negative_desired_rpm_if_currently_positive_rpm)
{
    ON_CALL(sub, getDesiredRpm).WillByDefault(Return(SentinelDriveEvadeCommand::MIN_RPM));
    EXPECT_CALL(sub, setDesiredRpm(-SentinelDriveEvadeCommand::MIN_RPM));
    cmd.initialize();
}

TEST_F(SentinelDriveEvadeCommandTest, end_sets_desired_rpm_to_0)
{
    EXPECT_CALL(sub, setDesiredRpm(FloatNear(0, 1E-5))).Times(2);
    cmd.end(true);
    cmd.end(false);
}

TEST_F(SentinelDriveEvadeCommandTest, execute_direction_changes_immediately_when_at_start)
{
    position = 0;

    cmd.initialize();

    desiredRpm = -1000;

    cmd.execute();

    EXPECT_NEAR(SentinelDriveEvadeCommand::MIN_RPM, desiredRpm, 1E-5);
}

TEST_F(SentinelDriveEvadeCommandTest, execute_direction_changes_immediately_when_at_end)
{
    position = SentinelDriveSubsystem::RAIL_LENGTH - SentinelDriveSubsystem::SENTINEL_LENGTH;

    cmd.initialize();

    desiredRpm = 1000;

    cmd.execute();

    EXPECT_NEAR(-SentinelDriveEvadeCommand::MIN_RPM, desiredRpm, 1E-5);
}

TEST_F(SentinelDriveEvadeCommandTest, execute_direction_changes_immediately_when_close_to_start)
{
    position = SentinelDriveEvadeCommand::TURNAROUND_BUFFER;

    cmd.initialize();

    desiredRpm = -1000;

    cmd.execute();

    EXPECT_NEAR(SentinelDriveEvadeCommand::MIN_RPM, desiredRpm, 1E-5);
}

TEST_F(SentinelDriveEvadeCommandTest, execute_direction_changes_immediately_when_close_to_end)
{
    position = SentinelDriveSubsystem::RAIL_LENGTH - SentinelDriveSubsystem::SENTINEL_LENGTH -
               SentinelDriveEvadeCommand::TURNAROUND_BUFFER;

    cmd.initialize();

    desiredRpm = 1000;

    cmd.execute();

    EXPECT_NEAR(-SentinelDriveEvadeCommand::MIN_RPM, desiredRpm, 1E-5);
}

TEST_F(SentinelDriveEvadeCommandTest, execute_direction_changes_after_some_distance_traveled)
{
    position = 1;
    if (SentinelDriveEvadeCommand::TURNAROUND_BUFFER >
        SentinelDriveEvadeCommand::LARGE_ARMOR_PLATE_WIDTH)
    {
        position += SentinelDriveEvadeCommand::TURNAROUND_BUFFER -
                    SentinelDriveEvadeCommand::LARGE_ARMOR_PLATE_WIDTH;
    }
    else
    {
        position += SentinelDriveEvadeCommand::LARGE_ARMOR_PLATE_WIDTH;
    }

    desiredRpm = -1;
    cmd.initialize();

    EXPECT_NEAR(SentinelDriveEvadeCommand::LARGE_ARMOR_PLATE_WIDTH, cmd.getDistanceToDrive(), 1E-5);

    cmd.execute();

    EXPECT_NEAR(SentinelDriveEvadeCommand::MIN_RPM, desiredRpm, 1E-5);

    position += SentinelDriveEvadeCommand::LARGE_ARMOR_PLATE_WIDTH / 2;

    cmd.execute();

    EXPECT_NEAR(SentinelDriveEvadeCommand::MIN_RPM, desiredRpm, 1E-5);

    position += SentinelDriveEvadeCommand::LARGE_ARMOR_PLATE_WIDTH / 2;

    cmd.execute();

    EXPECT_NEAR(-SentinelDriveEvadeCommand::MIN_RPM, desiredRpm, 1E-5);
}

TEST_F(
    SentinelDriveEvadeCommandTest,
    execute_direction_changes_negative_to_positive_after_distance_traveled)
{
    position = SentinelDriveSubsystem::RAIL_LENGTH - SentinelDriveSubsystem::SENTINEL_LENGTH - 1;
    if (SentinelDriveEvadeCommand::TURNAROUND_BUFFER >
        SentinelDriveEvadeCommand::LARGE_ARMOR_PLATE_WIDTH)
    {
        position -= SentinelDriveEvadeCommand::TURNAROUND_BUFFER -
                    SentinelDriveEvadeCommand::LARGE_ARMOR_PLATE_WIDTH;
    }
    else
    {
        position -= SentinelDriveEvadeCommand::LARGE_ARMOR_PLATE_WIDTH;
    }

    desiredRpm = 1;
    cmd.initialize();

    EXPECT_NEAR(SentinelDriveEvadeCommand::LARGE_ARMOR_PLATE_WIDTH, cmd.getDistanceToDrive(), 1E-5);

    cmd.execute();

    EXPECT_NEAR(-SentinelDriveEvadeCommand::MIN_RPM, desiredRpm, 1E-5);

    position -= SentinelDriveEvadeCommand::LARGE_ARMOR_PLATE_WIDTH / 2;

    cmd.execute();

    EXPECT_NEAR(-SentinelDriveEvadeCommand::MIN_RPM, desiredRpm, 1E-5);

    position -= SentinelDriveEvadeCommand::LARGE_ARMOR_PLATE_WIDTH / 2;

    cmd.execute();

    EXPECT_NEAR(SentinelDriveEvadeCommand::MIN_RPM, desiredRpm, 1E-5);
}

TEST_F(
    SentinelDriveEvadeCommandTest,
    execute_distance_to_travel_larger_when_turning_around_inside_turnaround_buffer_near_start_of_rail)
{
    position = 0;
    cmd.initialize();

    desiredRpm = -1000;

    cmd.execute();

    EXPECT_NEAR(
        (SentinelDriveSubsystem::RAIL_LENGTH - SentinelDriveSubsystem::SENTINEL_LENGTH) / 2,
        cmd.getDistanceToDrive(),
        1E-5);

    desiredRpm = -1000;
    position = SentinelDriveEvadeCommand::TURNAROUND_BUFFER / 2;

    cmd.execute();

    EXPECT_NEAR(
        (SentinelDriveSubsystem::RAIL_LENGTH - SentinelDriveSubsystem::SENTINEL_LENGTH) / 2 -
            position,
        cmd.getDistanceToDrive(),
        1E-5);
}

TEST_F(
    SentinelDriveEvadeCommandTest,
    execute_distance_to_travel_larger_when_turning_around_inside_turnaround_buffer_near_end_of_rail)
{
    position = SentinelDriveSubsystem::RAIL_LENGTH - SentinelDriveSubsystem::SENTINEL_LENGTH;
    cmd.initialize();

    desiredRpm = 1000;

    cmd.execute();

    EXPECT_NEAR(
        (SentinelDriveSubsystem::RAIL_LENGTH - SentinelDriveSubsystem::SENTINEL_LENGTH) / 2,
        cmd.getDistanceToDrive(),
        1E-5);

    desiredRpm = 1000;
    position -= SentinelDriveEvadeCommand::TURNAROUND_BUFFER / 2;

    cmd.execute();

    EXPECT_NEAR(
        (SentinelDriveSubsystem::RAIL_LENGTH - SentinelDriveSubsystem::SENTINEL_LENGTH) / 2 -
            SentinelDriveEvadeCommand::TURNAROUND_BUFFER / 2,
        cmd.getDistanceToDrive(),
        1E-5);
}
