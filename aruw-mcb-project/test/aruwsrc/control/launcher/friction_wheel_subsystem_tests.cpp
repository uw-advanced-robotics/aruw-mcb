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

tap::communication::serial::RefSerial::Rx::RobotData ROBOT_DATA{};

class FrictionWheelSubsystemTest : public Test
{
protected:
    FrictionWheelSubsystemTest()
        : leftFlywheel(
              &drivers,
              tap::motor::MOTOR1,
              tap::can::CanBus::CAN_BUS1,
              true,
              "Left flywheel",
              false),
          rightFlywheel(
              &drivers,
              tap::motor::MOTOR2,
              tap::can::CanBus::CAN_BUS1,
              false,
              "Right flywheel",
              false),
          thirdFlywheel(
              &drivers,
              tap::motor::MOTOR3,
              tap::can::CanBus::CAN_BUS1,
              false,
              "Third flywheel",
              false),
          frictionWheels(
              &drivers,
              std::array<tap::motor::MotorInterface*, 2>{{&leftFlywheel, &rightFlywheel}},
              WHEEL_CONFIGS_ARRAY,
              nullptr),
          tripleFrictionWheels(
              &drivers,
              std::array<tap::motor::MotorInterface*, 3>{
                  {&leftFlywheel, &rightFlywheel, &thirdFlywheel}},
              WHEEL_CONFIG,
              nullptr)
    {
    }

    ClockStub clock;
    tap::Drivers drivers;
    NiceMock<tap::mock::DjiMotorMock> leftFlywheel;
    NiceMock<tap::mock::DjiMotorMock> rightFlywheel;
    NiceMock<tap::mock::DjiMotorMock> thirdFlywheel;
    std::array<FlywheelConfig, 2> WHEEL_CONFIGS_ARRAY = {WHEEL_CONFIG, WHEEL_CONFIG};
    FrictionWheelSubsystem<2> frictionWheels;
    FrictionWheelSubsystem<3> tripleFrictionWheels;
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
    ON_CALL(leftFlywheel.getInternalEncoder(), getShaftRPM).WillByDefault(Return(0));
    EXPECT_CALL(leftFlywheel, setDesiredOutput(0)).Times(2);
    ON_CALL(rightFlywheel.getInternalEncoder(), getShaftRPM).WillByDefault(Return(0));
    EXPECT_CALL(rightFlywheel, setDesiredOutput(0)).Times(2);
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(ROBOT_DATA));

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
    ON_CALL(leftFlywheel.getInternalEncoder(), getShaftRPM).WillByDefault(Return(0));
    EXPECT_CALL(leftFlywheel, setDesiredOutput(Gt(0)));
    ON_CALL(rightFlywheel.getInternalEncoder(), getShaftRPM).WillByDefault(Return(0));
    EXPECT_CALL(rightFlywheel, setDesiredOutput(Gt(0)));
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(ROBOT_DATA));

    frictionWheels.setDesiredLaunchSpeed(10);

    clock.time = 0;
    frictionWheels.initialize();

    clock.time = 1;
    frictionWheels.refresh();
}

TEST_F(FrictionWheelSubsystemTest, refresh__negative_output_when_desired_speed_0_shaft_rpm_negative)
{
    ON_CALL(leftFlywheel.getInternalEncoder(), getShaftRPM).WillByDefault(Return(1000));
    EXPECT_CALL(leftFlywheel, setDesiredOutput(Lt(0)));
    ON_CALL(rightFlywheel.getInternalEncoder(), getShaftRPM).WillByDefault(Return(1000));
    EXPECT_CALL(rightFlywheel, setDesiredOutput(Lt(0)));
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(ROBOT_DATA));

    clock.time = 0;
    frictionWheels.initialize();

    clock.time = 1;
    frictionWheels.refresh();
}

TEST_F(FrictionWheelSubsystemTest, refresh_updates_desiredRpmRamp_when_target_not_reached)
{
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(ROBOT_DATA));

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
        EXPECT_NEAR(frictionWheels.desiredRpmRamp.getTarget(), middleRpm, 1E-1);
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

TEST_F(
    FrictionWheelSubsystemTest,
    changeWheelVelocityState__wheel_zero_changed_to_zero_rpm_wheel_one_normal)
{
    ON_CALL(leftFlywheel.getInternalEncoder(), getShaftRPM).WillByDefault(Return(0));
    EXPECT_CALL(leftFlywheel, setDesiredOutput(0)).Times(2);
    ON_CALL(rightFlywheel.getInternalEncoder(), getShaftRPM).WillByDefault(Return(0));
    EXPECT_CALL(rightFlywheel, setDesiredOutput(0));
    EXPECT_CALL(rightFlywheel, setDesiredOutput(Gt(0)));  // wheel one positive movement
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(ROBOT_DATA));

    clock.time = 0;
    frictionWheels.initialize();

    clock.time = 1;
    frictionWheels.refresh();

    clock.time = 2;
    frictionWheels.setIndividualVelocity(0, 0);
    frictionWheels.changeWheelVelocityState(0, true);
    frictionWheels.setDesiredLaunchSpeed(10);
    frictionWheels.refresh();
}

TEST_F(
    FrictionWheelSubsystemTest,
    changeWheelVelocityState__wheel_zero_changed_to_negative_rpm_wheel_one_normal)
{
    ON_CALL(leftFlywheel.getInternalEncoder(), getShaftRPM).WillByDefault(Return(0));
    EXPECT_CALL(leftFlywheel, setDesiredOutput(0));
    EXPECT_CALL(
        leftFlywheel,
        setDesiredOutput(Lt(0)));  // wheel zero negative movement
    ON_CALL(rightFlywheel.getInternalEncoder(), getShaftRPM).WillByDefault(Return(0));
    EXPECT_CALL(rightFlywheel, setDesiredOutput(0));
    EXPECT_CALL(rightFlywheel, setDesiredOutput(Gt(0)));  // wheel one positive movement
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(ROBOT_DATA));

    clock.time = 0;
    frictionWheels.initialize();

    clock.time = 1;
    frictionWheels.refresh();

    clock.time = 2;
    frictionWheels.setIndividualVelocity(0, -60);
    frictionWheels.changeWheelVelocityState(0, true);
    frictionWheels.setDesiredLaunchSpeed(10);
    frictionWheels.refresh();
}

// TEST_F(FrictionWheelSubsystemTest, modularFlywheel_triple_wheel_movement)
// {
//     ON_CALL(tripleLeftFlywheel->getInternalEncoder(), getShaftRPM)
//         .WillByDefault(Return(0));
//     EXPECT_CALL(*tripleLeftFlywheel, setDesiredOutput(0));
//     EXPECT_CALL(*tripleLeftFlywheel, setDesiredOutput(Gt(0)));
//     ON_CALL(tripleFrictionWheels.wheels[1]->getInternalEncoder(), getShaftRPM)
//         .WillByDefault(Return(0));
//     EXPECT_CALL(*tripleFrictionWheels.wheels[1], setDesiredOutput(0));
//     EXPECT_CALL(*tripleFrictionWheels.wheels[1], setDesiredOutput(Gt(0)));
//     ON_CALL(tripleFrictionWheels.wheels[2]->getInternalEncoder(), getShaftRPM)
//         .WillByDefault(Return(0));
//     EXPECT_CALL(*tripleFrictionWheels.wheels[2], setDesiredOutput(0));
//     EXPECT_CALL(*tripleFrictionWheels.wheels[2], setDesiredOutput(Gt(0)));
//     ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(ROBOT_DATA));

//     clock.time = 0;
//     tripleFrictionWheels.initialize();

//     clock.time = 1;
//     tripleFrictionWheels.refresh();

//     clock.time = 2;
//     tripleFrictionWheels.setDesiredLaunchSpeed(10);
//     tripleFrictionWheels.refresh();
// }