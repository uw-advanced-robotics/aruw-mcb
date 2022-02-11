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

#include "aruwsrc/control/launcher/friction_wheel_subsystem.hpp"
#include "aruwsrc/drivers.hpp"

using aruwsrc::Drivers;
using namespace testing;
using namespace aruwsrc::control::launcher;

#define SETUP_TEST() \
    Drivers drivers; \
    FrictionWheelSubsystem frictionWheels(&drivers);

TEST(FrictionWheelSubsystem, onHardwareTestStart__sets_desired_speed_nonzero)
{
    SETUP_TEST();
    frictionWheels.onHardwareTestStart();
    EXPECT_NEAR(15.0f, frictionWheels.getDesiredLaunchSpeed(), 1E-3);
}

TEST(FrictionWheelSubsystem, onHardwareTestComplete__sets_desired_speed_zero)
{
    SETUP_TEST();
    frictionWheels.onHardwareTestComplete();
    EXPECT_NEAR(0.0f, frictionWheels.getDesiredLaunchSpeed(), 1E-3);
}

TEST(FrictionWheelSubsystem, refresh__0_output_when_desired_speed_0_shaft_rpm_0)
{
    SETUP_TEST();

    ON_CALL(frictionWheels.leftWheel, getShaftRPM()).WillByDefault(Return(0));
    EXPECT_CALL(frictionWheels.leftWheel, setDesiredOutput(0)).Times(2);
    ON_CALL(frictionWheels.rightWheel, getShaftRPM()).WillByDefault(Return(0));
    EXPECT_CALL(frictionWheels.rightWheel, setDesiredOutput(0)).Times(2);

    tap::arch::clock::setTime(0);
    frictionWheels.initialize();

    tap::arch::clock::setTime(1);
    frictionWheels.refresh();

    tap::arch::clock::setTime(2);
    frictionWheels.setDesiredLaunchSpeed(0);
    frictionWheels.refresh();
}

TEST(FrictionWheelSubsystem, refresh__positive_output_when_desired_speed_10_shaft_rpm_0)
{
    SETUP_TEST();

    ON_CALL(frictionWheels.leftWheel, getShaftRPM()).WillByDefault(Return(0));
    EXPECT_CALL(frictionWheels.leftWheel, setDesiredOutput(Gt(0)));
    ON_CALL(frictionWheels.rightWheel, getShaftRPM()).WillByDefault(Return(0));
    EXPECT_CALL(frictionWheels.rightWheel, setDesiredOutput(Gt(0)));

    frictionWheels.setDesiredLaunchSpeed(10);

    tap::arch::clock::setTime(0);
    frictionWheels.initialize();

    tap::arch::clock::setTime(1);
    frictionWheels.refresh();
}

TEST(FrictionWheelSubsystem, refresh__negative_output_when_desired_speed_0_shaft_rpm_negative)
{
    SETUP_TEST();

    ON_CALL(frictionWheels.leftWheel, getShaftRPM()).WillByDefault(Return(1000));
    EXPECT_CALL(frictionWheels.leftWheel, setDesiredOutput(Lt(0)));
    ON_CALL(frictionWheels.rightWheel, getShaftRPM()).WillByDefault(Return(1000));
    EXPECT_CALL(frictionWheels.rightWheel, setDesiredOutput(Lt(0)));

    tap::arch::clock::setTime(0);
    frictionWheels.initialize();

    tap::arch::clock::setTime(1);
    frictionWheels.refresh();
}

TEST(FrictionWheelSubsystem, refresh_updates_desiredRpmRamp_when_target_not_reached)
{
    SETUP_TEST();

    frictionWheels.setDesiredLaunchSpeed(
        FrictionWheelSubsystem::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT
            [MODM_ARRAY_SIZE(FrictionWheelSubsystem::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT) - 1]
                .first);

    uint32_t time = 0;
    tap::arch::clock::setTime(time);
    float prevRpmTarget = frictionWheels.desiredRpmRamp.getValue();

    for (int i = 0; i < 1000; i++)
    {
        time += 10;
        tap::arch::clock::setTime(time);
        frictionWheels.refresh();

        if (!frictionWheels.desiredRpmRamp.isTargetReached())
        {
            EXPECT_NE(prevRpmTarget, frictionWheels.desiredRpmRamp.getValue());
            prevRpmTarget = frictionWheels.desiredRpmRamp.getValue();
        }
    }
}

TEST(
    FrictionWheelSubsystem,
    setDesiredLaunchSpeed__setting_to_values_in_LUT_updates_rpm_ramp_correctly)
{
    SETUP_TEST();

    for (size_t i = 0;
         i < MODM_ARRAY_SIZE(FrictionWheelSubsystem::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT);
         i++)
    {
        const auto& tuple = FrictionWheelSubsystem::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[i];
        frictionWheels.setDesiredLaunchSpeed(tuple.first);
        EXPECT_NEAR(frictionWheels.desiredRpmRamp.getTarget(), tuple.second, 1E-3);
    }
}

TEST(
    FrictionWheelSubsystem,
    setDesiredLaunchSpeed__setting_to_value_in_between_LUT_updates_rpm_ramp_correctly)
{
    if (MODM_ARRAY_SIZE(FrictionWheelSubsystem::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT) == 0)
        return;

    SETUP_TEST();

    for (size_t i = 0;
         i < MODM_ARRAY_SIZE(FrictionWheelSubsystem::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT) - 1;
         i++)
    {
        const auto& firstTuple = FrictionWheelSubsystem::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[i];
        const auto& secondTuple =
            FrictionWheelSubsystem::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[i + 1];

        float middleSpeed = (firstTuple.first + secondTuple.first) / 2.0f;
        float middleRpm = (firstTuple.second + secondTuple.second) / 2.0f;

        frictionWheels.setDesiredLaunchSpeed(middleSpeed);
        EXPECT_NEAR(frictionWheels.desiredRpmRamp.getTarget(), middleRpm, 1E-3);
    }
}

TEST(FrictionWheelSubsystem, setDesiredLaunchSpeed__negative_launch_speed_0_desired_rpm)
{
    SETUP_TEST();

    frictionWheels.setDesiredLaunchSpeed(-100);

    EXPECT_EQ(0, frictionWheels.desiredRpmRamp.getTarget());
}

TEST(
    FrictionWheelSubsystem,
    setDesiredLaunchSpeed__speed_above_max_speed_set_launch_speed_to_max_rpm)
{
    SETUP_TEST();

    const auto& tuple = FrictionWheelSubsystem::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT
        [MODM_ARRAY_SIZE(FrictionWheelSubsystem::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT) - 1];

    frictionWheels.setDesiredLaunchSpeed(tuple.first + 10);

    EXPECT_EQ(tuple.second, frictionWheels.desiredRpmRamp.getTarget());
}
