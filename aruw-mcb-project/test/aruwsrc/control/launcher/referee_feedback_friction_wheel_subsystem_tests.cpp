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
#include "tap/drivers.hpp"

#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"

using namespace aruwsrc;
using namespace aruwsrc::control::launcher;
using namespace testing;

class RefereeFeedbackFrictionWheelSubsystemTest : public Test
{
protected:
    NiceMock<tap::mock::DjiMotorMock> leftFlywheel, rightFlywheel;
    std::array<FlywheelConfig, 2> wheelConfigs = {wheelConfigLeft, wheelConfigRight};
    RefereeFeedbackFrictionWheelSubsystemTest()
        : leftFlywheel(
              &drivers,
              tap::motor::MOTOR1,
              tap::can::CanBus::CAN_BUS1,
              true,
              "Left flywheel",
              false,
              1.0f,
              0u,
              static_cast<tap::encoder::EncoderInterface*>(nullptr)),
          rightFlywheel(
              &drivers,
              tap::motor::MOTOR2,
              tap::can::CanBus::CAN_BUS1,
              false,
              "Right flywheel",
              false,
              1.0f,
              0u,
              static_cast<tap::encoder::EncoderInterface*>(nullptr)),
          frictionWheels(
              &drivers,
              std::array<NiceMock<tap::mock::DjiMotorMock>*, 2>{{&leftFlywheel, &rightFlywheel}},
              wheelConfigs,
              tap::can::CanBus::CAN_BUS1,
              nullptr,
              tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1)
    {
         std::cout << "made to test" << std::endl;
    }

    void SetUp() override
    {
        ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));
        std::cout << "made to setup" << std::endl;
    }

    tap::arch::clock::ClockStub clock;
    tap::Drivers drivers;
    RefereeFeedbackFrictionWheelSubsystem<10, 2> frictionWheels;
    tap::communication::serial::RefSerialData::Rx::RobotData robotData;
};

TEST_F(
    RefereeFeedbackFrictionWheelSubsystemTest,
    getPredictedLaunchSpeed_same_as_desired_launch_speed_when_ref_system_offline)
{
    std::cout << "first test" << std::endl;
    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(false));
    std::cout << "line 1" << std::endl;
    frictionWheels.setDesiredLaunchSpeed(LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[0].first);
    std::cout << "line 2" << std::endl;
    frictionWheels.refresh();
    std::cout << "line 3" << std::endl;
    EXPECT_EQ(frictionWheels.getDesiredLaunchSpeed(), frictionWheels.getPredictedLaunchSpeed());
    std::cout << "line 4" << std::endl;
    frictionWheels.setDesiredLaunchSpeed(LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[1].first);
    std::cout << "line 5" << std::endl;
    frictionWheels.refresh();
    std::cout << "line 6" << std::endl;
    EXPECT_EQ(frictionWheels.getDesiredLaunchSpeed(), frictionWheels.getPredictedLaunchSpeed());
    EXPECT_TRUE(false);
    std::cout << "line 7" << std::endl;
}

TEST_F(
    RefereeFeedbackFrictionWheelSubsystemTest,
    getPredictedLaunchSpeed_launch_speed_based_on_ref_system_measured_bullet_speed)
{
    std::cout << "test 2" << std::endl;
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
    std::cout << "test 2" << std::endl;
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
    std::cout << "test 2" << std::endl;
    RefereeFeedbackFrictionWheelSubsystem<10, 2> frictionWheelAveraged(
        &drivers,
        std::array<NiceMock<tap::mock::DjiMotorMock>*, 2>{{&leftFlywheel, &rightFlywheel}},
        wheelConfigs,
        tap::can::CanBus::CAN_BUS1,
        nullptr,
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
