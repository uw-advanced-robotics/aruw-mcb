/*
 * Copyright (c) 2026-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/sensors/current/analog_current_sensor.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/robot/engineer/algorithms/inverse_kinematics/abstract_ik_command.hpp"

using namespace testing;

class InverseKinematicsTests : public Test
{
protected:
    InverseKinematicsTests()
        : currentSensor(
              {&drivers.analog,
               aruwsrc::control::chassis::CURRENT_SENSOR_PIN,
               aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_MV_PER_MA,
               aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_ZERO_MA,
               aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_LOW_PASS_ALPHA}),
          voltageSensor(),
          leftFrontMotor(),
          leftBackMotor(),
          rightFrontMotor(),
          rightBackMotor(),
          chassis(
              &drivers,
              &currentSensor,
              &voltageSensor,
              leftFrontMotor,
              leftBackMotor,
              rightFrontMotor,
              rightBackMotor,
              MOCK_WHEEL_VELOCITY_PID_CONFIG,
              WHEEL_RADIUS,
              WHEELBASE_RADIUS)
    {
    }

    void SetUp() override
    {
        ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(testing::Return(false));
        ON_CALL(drivers.refSerial, getRobotData).WillByDefault(testing::ReturnRef(robotData));
    }

    tap::Drivers drivers;
    tap::communication::sensors::current::AnalogCurrentSensor currentSensor;
    aruwsrc::communication::sensors::voltage::FakeVoltageSensor voltageSensor;
    NiceMock<tap::mock::MotorInterfaceMock> leftFrontMotor, leftBackMotor, rightFrontMotor,
        rightBackMotor;
    XDriveChassisSubsystem chassis;
    tap::communication::serial::RefSerialData::Rx::RobotData robotData;
};
