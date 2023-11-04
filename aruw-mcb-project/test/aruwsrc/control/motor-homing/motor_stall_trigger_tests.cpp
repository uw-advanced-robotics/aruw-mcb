/*
 * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/control/homeable-subsystem/trigger/motor_stall_trigger.hpp"
#include "tap/mock/dji_motor_mock.hpp"

using namespace aruwsrc::control;
using namespace testing;

class MotorStallTriggerTest : public Test
{
protected:
    MotorStallTriggerTest()
        : motor(&drivers, tap::motor::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "mock motor"),
          trigger(motor, maxRPM, minTorque)
    {
    }

    tap::Drivers drivers;
    NiceMock<tap::mock::DjiMotorMock> motor;
    const int16_t maxRPM = 100;
    const int16_t minTorque = 10;
    MotorStallTrigger trigger;
};

TEST_F(MotorStallTriggerTest, torque_in_rpm_out_no_stall)
{
    int16_t rpm;
    int16_t torque;

    ON_CALL(motor, getShaftRPM).WillByDefault(ReturnPointee(&rpm));
    ON_CALL(motor, getTorque).WillByDefault(ReturnPointee(&torque));
    

    rpm = maxRPM + 1;
    torque = minTorque - 1;
    EXPECT_EQ(false, trigger.isTriggered());

    rpm = -maxRPM - 1;
    torque = -minTorque + 1;
    EXPECT_EQ(false, trigger.isTriggered());
}


TEST_F(MotorStallTriggerTest, torque_out_rpm_in_stall)
{
    int16_t rpm;
    int16_t torque;

    ON_CALL(motor, getShaftRPM).WillByDefault(ReturnPointee(&rpm));
    ON_CALL(motor, getTorque).WillByDefault(ReturnPointee(&torque));

    rpm = maxRPM - 1;
    torque = minTorque + 1;
    EXPECT_EQ(true, trigger.isTriggered());

    rpm = -maxRPM + 1;
    torque = -minTorque - 1;
    EXPECT_EQ(true, trigger.isTriggered());
}

TEST_F(MotorStallTriggerTest, torque_in_rpm_in_no_stall) {
    int16_t rpm;
    int16_t torque;

    ON_CALL(motor, getShaftRPM).WillByDefault(ReturnPointee(&rpm));
    ON_CALL(motor, getTorque).WillByDefault(ReturnPointee(&torque));

    rpm = maxRPM - 1;
    torque = minTorque - 1;
    EXPECT_EQ(false, trigger.isTriggered());

    rpm = -maxRPM + 1;
    torque = -minTorque + 1;
    EXPECT_EQ(false, trigger.isTriggered());
}

TEST_F(MotorStallTriggerTest, torque_out_rpm_out_no_stall) {
    int16_t rpm;
    int16_t torque;

    ON_CALL(motor, getShaftRPM).WillByDefault(ReturnPointee(&rpm));
    ON_CALL(motor, getTorque).WillByDefault(ReturnPointee(&torque));
    
    rpm = maxRPM + 1;
    torque = minTorque + 1;
    EXPECT_EQ(false, trigger.isTriggered());

    rpm = -maxRPM - 1;
    torque = -minTorque - 1;
    EXPECT_EQ(false, trigger.isTriggered());
}