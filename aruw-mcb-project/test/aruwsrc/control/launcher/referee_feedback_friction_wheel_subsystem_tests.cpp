#include <gtest/gtest.h>

#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/drivers.hpp"

using namespace aruwsrc;
using namespace aruwsrc::control::launcher;
using namespace testing;

#define SETUP_TEST()                                                                \
    Drivers drivers;                                                                \
    RefereeFeedbackFrictionWheelSubsystem frictionWheels(                           \
        &drivers,                                                                   \
        tap::motor::MOTOR1,                                                         \
        tap::motor::MOTOR2,                                                         \
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1); \
    tap::communication::serial::RefSerialData::Rx::RobotData robotData;             \
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));

// Refresh this many times to compensate for averaging
static constexpr size_t TIMES_TO_REFRESH = 100;

TEST(
    RefereeFeedbackFrictionWheelSubsystemTests,
    getPredictedLaunchSpeed_same_as_desired_launch_speed_when_ref_system_offline)
{
    SETUP_TEST();
    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(false));

    for (size_t i = 0; i < TIMES_TO_REFRESH; i++)
    {
        frictionWheels.refresh();
    }

    frictionWheels.setDesiredLaunchSpeed(
        FrictionWheelSubsystem::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[0].first);
    EXPECT_EQ(frictionWheels.getDesiredLaunchSpeed(), frictionWheels.getPredictedLaunchSpeed());

    for (size_t i = 0; i < TIMES_TO_REFRESH; i++)
    {
        frictionWheels.refresh();
    }

    frictionWheels.setDesiredLaunchSpeed(
        FrictionWheelSubsystem::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[1].first);
    EXPECT_EQ(frictionWheels.getDesiredLaunchSpeed(), frictionWheels.getPredictedLaunchSpeed());
}

TEST(
    RefereeFeedbackFrictionWheelSubsystemTests,
    getPredictedLaunchSpeed_launch_speed_based_on_ref_system_measured_bullet_speed)
{
    SETUP_TEST();
    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(true));

    frictionWheels.setDesiredLaunchSpeed(
        FrictionWheelSubsystem::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[0].first);

    for (size_t i = 0; i < TIMES_TO_REFRESH; i++)
    {
        frictionWheels.refresh();
    }

    frictionWheels.setDesiredLaunchSpeed(
        FrictionWheelSubsystem::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[0].first);
}

TEST(RefereeFeedbackFrictionWheelSubsystemTests, F) {}
