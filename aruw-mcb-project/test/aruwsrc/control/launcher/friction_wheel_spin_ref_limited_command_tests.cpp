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

#include "aruwsrc/control/launcher/friction_wheel_spin_ref_limited_command.hpp"
#include "aruwsrc/mock/friction_wheel_subsystem_mock.hpp"

using aruwsrc::control::launcher::FrictionWheelSpinRefLimitedCommand;
using tap::Drivers;
using namespace testing;
using namespace tap::communication::serial;

class FrictionWheelSpinRefLimitedCommandTest : public Test
{
protected:
    FrictionWheelSpinRefLimitedCommandTest() : frictionWheels(&drivers) {}

    void SetUp() override
        ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));
        ON_CALL(drivers.refSerial, getRefSerialReceivingData)
            .WillByDefault(ReturnPointee(&refSerialOnline));
    }

    tap::Drivers drivers;
    aruwsrc::mock::FrictionWheelSubsystemMock frictionWheels;
    RefSerialData::Rx::RobotData robotData{};
    bool refSerialOnline = false;
};

#define TEST_EXECUTE(name) TEST_F(FrictionWheelSpinRefLimitedCommandTest, execute__##name)

TEST_EXECUTE(defaultLaunchSpeed_used_when_ref_serial_offline)
{
    FrictionWheelSpinRefLimitedCommand frictionWheelSpinRefLimitedCommand(
        &drivers,
        &frictionWheels,
        5,
        false,
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

    refSerialOnline = false;
    EXPECT_CALL(frictionWheels, setDesiredLaunchSpeed(5));

    frictionWheelSpinRefLimitedCommand.execute();
}

TEST_EXECUTE(defaultLaunchSpeed_used_when_alwaysUseDefaultLaunchSpeed_true_ref_serial_online)
{
    FrictionWheelSpinRefLimitedCommand frictionWheelSpinRefLimitedCommand(
        &drivers,
        &frictionWheels,
        0,
        true,
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

    refSerialOnline = true;
    EXPECT_CALL(frictionWheels, setDesiredLaunchSpeed(0));

    frictionWheelSpinRefLimitedCommand.execute();
}

TEST_EXECUTE(defaultLaunchSpeed_used_when_alwaysUseDefaultLaunchSpeed_true_ref_serial_offline)
{
    FrictionWheelSpinRefLimitedCommand frictionWheelSpinRefLimitedCommand(
        &drivers,
        &frictionWheels,
        0,
        true,
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

    refSerialOnline = false;
    EXPECT_CALL(frictionWheels, setDesiredLaunchSpeed(0));

    frictionWheelSpinRefLimitedCommand.execute();
}

TEST_F(FrictionWheelSpinRefLimitedCommandTest, isFinished__always_false)
{
    FrictionWheelSpinRefLimitedCommand frictionWheelSpinRefLimitedCommand(
        &drivers,
        &frictionWheels,
        0,
        true,
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

    EXPECT_FALSE(frictionWheelSpinRefLimitedCommand.isFinished());
}

TEST_F(FrictionWheelSpinRefLimitedCommandTest, end__sets_launch_speed_to_0)
{
    FrictionWheelSpinRefLimitedCommand frictionWheelSpinRefLimitedCommand(
        &drivers,
        &frictionWheels,
        0,
        true,
        tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1);

    EXPECT_CALL(frictionWheels, setDesiredLaunchSpeed(0)).Times(2);

    frictionWheelSpinRefLimitedCommand.end(true);
    frictionWheelSpinRefLimitedCommand.end(false);
}
