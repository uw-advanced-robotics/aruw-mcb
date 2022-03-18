#include <gtest/gtest.h>

#include "tap/architecture/clock.hpp"

#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/drivers.hpp"

using namespace aruwsrc;
using namespace aruwsrc::control::launcher;
using namespace testing;

class RefereeFeedbackFrictionWheelSubsystemTest : public Test
{
protected:
    RefereeFeedbackFrictionWheelSubsystemTest()
        : frictionWheels(
              &drivers,
              tap::motor::MOTOR1,
              tap::motor::MOTOR2,
              tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1)
    {
    }

    void SetUp() override
    {
        ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));
    }

    tap::arch::clock::ClockStub clock;
    Drivers drivers;
    RefereeFeedbackFrictionWheelSubsystem frictionWheels;
    tap::communication::serial::RefSerialData::Rx::RobotData robotData;
};

// Refresh this many times to compensate for averaging
static constexpr size_t TIMES_TO_REFRESH = 100;

TEST_F(
    RefereeFeedbackFrictionWheelSubsystemTest,
    getPredictedLaunchSpeed_same_as_desired_launch_speed_when_ref_system_offline)
{
    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(false));

    frictionWheels.setDesiredLaunchSpeed(
        FrictionWheelSubsystem::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[0].first);

    frictionWheels.refresh();

    EXPECT_EQ(frictionWheels.getDesiredLaunchSpeed(), frictionWheels.getPredictedLaunchSpeed());

    frictionWheels.setDesiredLaunchSpeed(
        FrictionWheelSubsystem::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[1].first);

    frictionWheels.refresh();

    EXPECT_EQ(frictionWheels.getDesiredLaunchSpeed(), frictionWheels.getPredictedLaunchSpeed());
}

TEST_F(
    RefereeFeedbackFrictionWheelSubsystemTest,
    getPredictedLaunchSpeed_launch_speed_based_on_ref_system_measured_bullet_speed)
{
    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(true));

    frictionWheels.setDesiredLaunchSpeed(
        FrictionWheelSubsystem::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[0].first);

    robotData.turret.lastReceivedLaunchingInfoTimestamp = 0;
    robotData.turret.bulletSpeed =
        FrictionWheelSubsystem::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[0].first + 5.0f;
    robotData.turret.launchMechanismID =
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1;

    for (size_t i = 0; i < TIMES_TO_REFRESH; i++)
    {
        frictionWheels.refresh();
        clock.time += 1;
        robotData.turret.lastReceivedLaunchingInfoTimestamp += 1;
    }

    EXPECT_NEAR(robotData.turret.bulletSpeed, frictionWheels.getPredictedLaunchSpeed(), 1E-1);

    frictionWheels.setDesiredLaunchSpeed(
        FrictionWheelSubsystem::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[1].first);

    robotData.turret.bulletSpeed =
        FrictionWheelSubsystem::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[1].first + 5.0f;

    for (size_t i = 0; i < TIMES_TO_REFRESH; i++)
    {
        frictionWheels.refresh();
        clock.time += 1;
        robotData.turret.lastReceivedLaunchingInfoTimestamp += 1;
    }

    EXPECT_NEAR(robotData.turret.bulletSpeed, frictionWheels.getPredictedLaunchSpeed(), 1E-1);
}

TEST_F(
    RefereeFeedbackFrictionWheelSubsystemTest,
    getPredictedLaunchSpeed_does_not_update_when_lastReceivedLaunchingInfoTimestamp_does_not_change)
{
    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(true));

    robotData.turret.lastReceivedLaunchingInfoTimestamp = 0;
    robotData.turret.bulletSpeed =
        FrictionWheelSubsystem::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[0].first + 5.0f;
    robotData.turret.launchMechanismID =
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1;

    frictionWheels.setDesiredLaunchSpeed(
        FrictionWheelSubsystem::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[0].first);

    for (size_t i = 0; i < TIMES_TO_REFRESH; i++)
    {
        frictionWheels.refresh();
        clock.time += 1;
    }

    EXPECT_EQ(frictionWheels.getDesiredLaunchSpeed(), frictionWheels.getPredictedLaunchSpeed());
}
