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

#include "aruwsrc/control/launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/mock/friction_wheel_subsystem_mock.hpp"

using aruwsrc::Drivers;
using aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand;
using namespace testing;

#define SETUP_TEST(...)                                                    \
    Drivers drivers;                                                       \
    aruwsrc::mock::FrictionWheelSubsystemMock frictionWheels(&drivers);    \
    FrictionWheelSpinRefLimitedCommand frictionWheelSpinRefLimitedCommand( \
        &drivers,                                                          \
        &frictionWheels,                                                   \
        __VA_ARGS__);                                                      \
    tap::serial::RefSerialData::Rx::RobotData robotData{};                 \
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));

#define TEST_EXECUTE(name) TEST(FrictionWheelSpinRefLimitedCommand, execute__##name)

TEST_EXECUTE(defaultLaunchSpeed_used_when_ref_serial_offline)
{
    SETUP_TEST(5, false, FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM1);

    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(false));
    EXPECT_CALL(frictionWheels, setDesiredLaunchSpeed(5));

    frictionWheelSpinRefLimitedCommand.execute();
}

TEST_EXECUTE(barrelSpeedLimit17ID1_used_when_ref_serial_online_barrel_1_specified)
{
    SETUP_TEST(5, false, FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM1);

    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(true));
    EXPECT_CALL(frictionWheels, setDesiredLaunchSpeed(10));
    robotData.turret.barrelSpeedLimit17ID1 = 10;

    frictionWheelSpinRefLimitedCommand.execute();
}

TEST_EXECUTE(barrelSpeedLimit17ID1_used_when_ref_serial_online_barrel_2_specified)
{
    SETUP_TEST(5, false, FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM2);

    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(true));
    EXPECT_CALL(frictionWheels, setDesiredLaunchSpeed(10));
    robotData.turret.barrelSpeedLimit17ID2 = 10;

    frictionWheelSpinRefLimitedCommand.execute();
}

TEST_EXECUTE(barrelSpeedLimit17ID2_used_when_ref_serial_online_barrel_42mm_specified)
{
    SETUP_TEST(5, false, FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_42MM);

    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(true));
    EXPECT_CALL(frictionWheels, setDesiredLaunchSpeed(10));
    robotData.turret.barrelSpeedLimit42 = 10;

    frictionWheelSpinRefLimitedCommand.execute();
}

TEST_EXECUTE(defaultLaunchSpeed_used_when_alwaysUseDefaultLaunchSpeed_true_ref_serial_online)
{
    SETUP_TEST(0, true, FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM1);

    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(true));
    EXPECT_CALL(frictionWheels, setDesiredLaunchSpeed(0));
    robotData.turret.barrelSpeedLimit17ID1 = 10;

    frictionWheelSpinRefLimitedCommand.execute();
}

TEST_EXECUTE(defaultLaunchSpeed_used_when_alwaysUseDefaultLaunchSpeed_true_ref_serial_offline)
{
    SETUP_TEST(0, true, FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM1);

    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(false));
    EXPECT_CALL(frictionWheels, setDesiredLaunchSpeed(0));
    robotData.turret.barrelSpeedLimit17ID1 = 10;

    frictionWheelSpinRefLimitedCommand.execute();
}

TEST_EXECUTE(launch_speed_changes_when_ref_serial_online_and_barrel_speed_changes)
{
    SETUP_TEST(0, false, FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM1);

    bool refSerialOnline = false;

    ON_CALL(drivers.refSerial, getRefSerialReceivingData)
        .WillByDefault(ReturnPointee(&refSerialOnline));
    EXPECT_CALL(frictionWheels, setDesiredLaunchSpeed(0));
    EXPECT_CALL(frictionWheels, setDesiredLaunchSpeed(10));
    EXPECT_CALL(frictionWheels, setDesiredLaunchSpeed(15));
    EXPECT_CALL(frictionWheels, setDesiredLaunchSpeed(30));

    frictionWheelSpinRefLimitedCommand.execute();

    refSerialOnline = true;

    robotData.turret.barrelSpeedLimit17ID1 = 10;
    frictionWheelSpinRefLimitedCommand.execute();

    robotData.turret.barrelSpeedLimit17ID1 = 15;
    frictionWheelSpinRefLimitedCommand.execute();

    robotData.turret.barrelSpeedLimit17ID1 = 30;
    frictionWheelSpinRefLimitedCommand.execute();
}

TEST(FrictionWheelSpinRefLimitedCommand, isFinished__always_false)
{
    SETUP_TEST(0, false, FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM1);

    EXPECT_FALSE(frictionWheelSpinRefLimitedCommand.isFinished());
}

TEST(FrictionWheelSpinRefLimitedCommand, end__sets_launch_speed_to_0)
{
    SETUP_TEST(0, false, FrictionWheelSpinRefLimitedCommand::Barrel::BARREL_17MM1);

    EXPECT_CALL(frictionWheels, setDesiredLaunchSpeed(0)).Times(2);

    frictionWheelSpinRefLimitedCommand.end(true);
    frictionWheelSpinRefLimitedCommand.end(false);
}
