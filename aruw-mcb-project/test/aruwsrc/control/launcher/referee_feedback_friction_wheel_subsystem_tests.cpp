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
              tap::can::CanBus::CAN_BUS1,
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

TEST_F(
    RefereeFeedbackFrictionWheelSubsystemTest,
    getPredictedLaunchSpeed_same_as_desired_launch_speed_when_ref_system_offline)
{
    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(false));

    frictionWheels.setDesiredLaunchSpeed(LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[0].first);

    frictionWheels.refresh();

    EXPECT_EQ(frictionWheels.getDesiredLaunchSpeed(), frictionWheels.getPredictedLaunchSpeed());

    frictionWheels.setDesiredLaunchSpeed(LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[1].first);

    frictionWheels.refresh();

    EXPECT_EQ(frictionWheels.getDesiredLaunchSpeed(), frictionWheels.getPredictedLaunchSpeed());
}

TEST_F(
    RefereeFeedbackFrictionWheelSubsystemTest,
    getPredictedLaunchSpeed_launch_speed_based_on_ref_system_measured_bullet_speed)
{
    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(true));

    frictionWheels.setDesiredLaunchSpeed(LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[0].first);

    robotData.turret.lastReceivedLaunchingInfoTimestamp = 0;
    robotData.turret.bulletSpeed = LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[0].first + 5.0f;
    robotData.turret.launchMechanismID =
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1;
    robotData.turret.firingFreq = 1;

    robotData.turret.lastReceivedLaunchingInfoTimestamp += 1;
    frictionWheels.refresh();

    EXPECT_NEAR(robotData.turret.bulletSpeed, frictionWheels.getPredictedLaunchSpeed(), 1E-1);

    frictionWheels.setDesiredLaunchSpeed(LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[1].first);

    robotData.turret.bulletSpeed = LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[1].first + 5.0f;

    robotData.turret.lastReceivedLaunchingInfoTimestamp += 1;
    frictionWheels.refresh();

    EXPECT_NEAR(robotData.turret.bulletSpeed, frictionWheels.getPredictedLaunchSpeed(), 1E-1);
}

TEST_F(
    RefereeFeedbackFrictionWheelSubsystemTest,
    getPredictedLaunchSpeed_does_not_update_when_lastReceivedLaunchingInfoTimestamp_does_not_change)
{
    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(true));

    robotData.turret.lastReceivedLaunchingInfoTimestamp = 0;
    robotData.turret.bulletSpeed = LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[0].first + 5.0f;
    robotData.turret.launchMechanismID =
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1;

    frictionWheels.setDesiredLaunchSpeed(LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[0].first);

    frictionWheels.refresh();

    EXPECT_EQ(frictionWheels.getDesiredLaunchSpeed(), frictionWheels.getPredictedLaunchSpeed());
}

TEST_F(RefereeFeedbackFrictionWheelSubsystemTest, getPredictedLaunchSpeed_rolling_average)
{
    RefereeFeedbackFrictionWheelSubsystem frictionWheelAveraged(
        &drivers,
        tap::motor::MOTOR1,
        tap::motor::MOTOR2,
        tap::can::CanBus::CAN_BUS1,
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

    robotData.turret.lastReceivedLaunchingInfoTimestamp = 0;
    robotData.turret.bulletSpeed = LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[0].first;
    robotData.turret.firingFreq = 1;
    robotData.turret.launchMechanismID =
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1;

    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(true));

    frictionWheelAveraged.setDesiredLaunchSpeed(LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[0].first);

    frictionWheelAveraged.refresh();

    // initially the desired and predicted launch speeds are the same
    EXPECT_EQ(
        frictionWheelAveraged.getDesiredLaunchSpeed(),
        frictionWheelAveraged.getPredictedLaunchSpeed());

    robotData.turret.lastReceivedLaunchingInfoTimestamp++;
    robotData.turret.bulletSpeed += 10;
    frictionWheelAveraged.refresh();

    robotData.turret.bulletSpeed += 10;

    for (size_t i = 0; i < 10; i++)
    {
        robotData.turret.lastReceivedLaunchingInfoTimestamp++;
        frictionWheelAveraged.refresh();
    }

    EXPECT_EQ(robotData.turret.bulletSpeed, frictionWheelAveraged.getPredictedLaunchSpeed());

    frictionWheelAveraged.setDesiredLaunchSpeed(LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[1].first);

    frictionWheelAveraged.refresh();
    robotData.turret.lastReceivedLaunchingInfoTimestamp++;

    EXPECT_EQ(
        frictionWheelAveraged.getDesiredLaunchSpeed(),
        frictionWheelAveraged.getPredictedLaunchSpeed());
}
