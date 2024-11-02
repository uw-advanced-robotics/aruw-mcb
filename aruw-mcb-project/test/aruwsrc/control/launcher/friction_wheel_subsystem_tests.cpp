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

#include "aruwsrc/control/launcher/friction_wheel_subsystem.hpp"

using tap::Drivers;
using namespace testing;
using namespace tap::arch::clock;
using namespace aruwsrc::control::launcher;

class FrictionWheelSubsystemTest : public Test
{
protected:
    FrictionWheelSubsystemTest()
        : frictionWheels(
              &drivers,
              tap::motor::MOTOR1,
              tap::motor::MOTOR2,
              tap::can::CanBus::CAN_BUS1,
              nullptr)
    {
    }

    ClockStub clock;
    tap::Drivers drivers;
    FrictionWheelSubsystem frictionWheels;
};

TEST_F(FrictionWheelSubsystemTest, initalizingHardwareTestCommand__sets_desired_speed_nonzero)
{
    frictionWheels.setDesiredLaunchSpeed(0);
    frictionWheels.getTestCommand()->initialize();
    EXPECT_NEAR(15.0f, frictionWheels.getDesiredLaunchSpeed(), 1E-3);
}

TEST_F(FrictionWheelSubsystemTest, endingHardwareTestCommand__sets_desired_speed_zero)
{
    frictionWheels.setDesiredLaunchSpeed(15);
    frictionWheels.getTestCommand()->end(true);
    EXPECT_NEAR(0.0f, frictionWheels.getDesiredLaunchSpeed(), 1E-3);
}

TEST_F(FrictionWheelSubsystemTest, refresh__0_output_when_desired_speed_0_shaft_rpm_0)
{
    ON_CALL(frictionWheels.leftWheel, getShaftRPM).WillByDefault(Return(0));
    EXPECT_CALL(frictionWheels.leftWheel, setDesiredOutput(0)).Times(2);
    ON_CALL(frictionWheels.rightWheel, getShaftRPM).WillByDefault(Return(0));
    EXPECT_CALL(frictionWheels.rightWheel, setDesiredOutput(0)).Times(2);

    clock.time = 0;
    frictionWheels.initialize();

    clock.time = 1;
    frictionWheels.refresh();

    clock.time = 2;
    frictionWheels.setDesiredLaunchSpeed(0);
    frictionWheels.refresh();
}

TEST_F(FrictionWheelSubsystemTest, refresh__positive_output_when_desired_speed_10_shaft_rpm_0)
{
    ON_CALL(frictionWheels.leftWheel, getShaftRPM).WillByDefault(Return(0));
    EXPECT_CALL(frictionWheels.leftWheel, setDesiredOutput(Gt(0)));
    ON_CALL(frictionWheels.rightWheel, getShaftRPM).WillByDefault(Return(0));
    EXPECT_CALL(frictionWheels.rightWheel, setDesiredOutput(Gt(0)));

    frictionWheels.setDesiredLaunchSpeed(10);

    clock.time = 0;
    frictionWheels.initialize();

    clock.time = 1;
    frictionWheels.refresh();
}

TEST_F(FrictionWheelSubsystemTest, refresh__negative_output_when_desired_speed_0_shaft_rpm_negative)
{
    ON_CALL(frictionWheels.leftWheel, getShaftRPM).WillByDefault(Return(1000));
    EXPECT_CALL(frictionWheels.leftWheel, setDesiredOutput(Lt(0)));
    ON_CALL(frictionWheels.rightWheel, getShaftRPM).WillByDefault(Return(1000));
    EXPECT_CALL(frictionWheels.rightWheel, setDesiredOutput(Lt(0)));

    clock.time = 0;
    frictionWheels.initialize();

    clock.time = 1;
    frictionWheels.refresh();
}

TEST_F(FrictionWheelSubsystemTest, refresh_updates_desiredRpmRamp_when_target_not_reached)
{
    frictionWheels.setDesiredLaunchSpeed(
        LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT
            [MODM_ARRAY_SIZE(LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT) - 1]
                .first);

    uint32_t time = 0;
    clock.time = time;
    float prevRpmTarget = frictionWheels.desiredRpmRamp.getValue();

    for (int i = 0; i < 1000; i++)
    {
        time += 10;
        clock.time = time;
        frictionWheels.refresh();

        if (!frictionWheels.desiredRpmRamp.isTargetReached())
        {
            EXPECT_NE(prevRpmTarget, frictionWheels.desiredRpmRamp.getValue());
            prevRpmTarget = frictionWheels.desiredRpmRamp.getValue();
        }
    }
}

TEST_F(
    FrictionWheelSubsystemTest,
    setDesiredLaunchSpeed__setting_to_values_in_LUT_updates_rpm_ramp_correctly)
{
    for (size_t i = 0; i < MODM_ARRAY_SIZE(LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT); i++)
    {
        const auto& tuple = LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[i];
        frictionWheels.setDesiredLaunchSpeed(tuple.first);
        EXPECT_NEAR(frictionWheels.desiredRpmRamp.getTarget(), tuple.second, 1E-3);
    }
}

TEST_F(
    FrictionWheelSubsystemTest,
    setDesiredLaunchSpeed__setting_to_value_in_between_LUT_updates_rpm_ramp_correctly)
{
    if (MODM_ARRAY_SIZE(LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT) == 0) return;

    for (size_t i = 0; i < MODM_ARRAY_SIZE(LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT) - 1; i++)
    {
        const auto& firstTuple = LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[i];
        const auto& secondTuple = LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[i + 1];

        float middleSpeed = (firstTuple.first + secondTuple.first) / 2.0f;
        float middleRpm = (firstTuple.second + secondTuple.second) / 2.0f;

        frictionWheels.setDesiredLaunchSpeed(middleSpeed);
        EXPECT_NEAR(frictionWheels.desiredRpmRamp.getTarget(), middleRpm, 1E-3);
    }
}

TEST_F(FrictionWheelSubsystemTest, setDesiredLaunchSpeed__negative_launch_speed_0_desired_rpm)
{
    frictionWheels.setDesiredLaunchSpeed(-100);

    EXPECT_EQ(0, frictionWheels.desiredRpmRamp.getTarget());
}

TEST_F(
    FrictionWheelSubsystemTest,
    setDesiredLaunchSpeed__speed_above_max_speed_set_launch_speed_to_max_rpm)
{
    const auto& tuple = LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT
        [MODM_ARRAY_SIZE(LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT) - 1];

    frictionWheels.setDesiredLaunchSpeed(tuple.first + 10);

    EXPECT_EQ(tuple.second, frictionWheels.desiredRpmRamp.getTarget());
}
