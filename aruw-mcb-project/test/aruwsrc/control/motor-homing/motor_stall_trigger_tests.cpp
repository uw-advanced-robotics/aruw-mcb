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
#include "tap/mock/dji_motor_mock.hpp"

#include "aruwsrc/control/bounded-subsystem/trigger/motor_stall_trigger.hpp"

using namespace aruwsrc::control;
using namespace testing;

class MotorStallTriggerTest : public Test
{
protected:
    MotorStallTriggerTest()
        : motor(&drivers, tap::motor::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "mock motor"),
          trigger(motor, maxVelocity, minTorque)
    {
    }

    tap::Drivers drivers;
    NiceMock<tap::mock::DjiMotorMock> motor;
    const float maxVelocity = 100;
    const int16_t minTorque = 10;
    MotorStallTrigger trigger;
};

TEST_F(MotorStallTriggerTest, torque_in_velocity_out_no_stall)
{
    float velocity;
    int16_t torque;

    ON_CALL(motor.getInternalEncoder(), isOnline).WillByDefault(Return(true));
    ON_CALL(motor.getInternalEncoder(), getVelocity).WillByDefault(ReturnPointee(&velocity));
    ON_CALL(motor, getTorque).WillByDefault(ReturnPointee(&torque));

    velocity = (maxVelocity + 1);
    torque = minTorque - 1;
    EXPECT_EQ(false, trigger.isTriggered());

    velocity = (-maxVelocity - 1);
    torque = -minTorque + 1;
    EXPECT_EQ(false, trigger.isTriggered());
}

TEST_F(MotorStallTriggerTest, torque_out_velocity_in_stall)
{
    float velocity;
    int16_t torque;

    ON_CALL(motor.getInternalEncoder(), isOnline).WillByDefault(Return(true));
    ON_CALL(motor.getInternalEncoder(), getVelocity).WillByDefault(ReturnPointee(&velocity));
    ON_CALL(motor, getTorque).WillByDefault(ReturnPointee(&torque));

    velocity = (maxVelocity - 1);
    torque = minTorque + 1;
    EXPECT_EQ(true, trigger.isTriggered());

    velocity = (-maxVelocity + 1);
    torque = -minTorque - 1;
    EXPECT_EQ(true, trigger.isTriggered());
}

TEST_F(MotorStallTriggerTest, torque_in_velocity_in_no_stall)
{
    float velocity;
    int16_t torque;

    ON_CALL(motor.getInternalEncoder(), isOnline).WillByDefault(Return(true));
    ON_CALL(motor.getInternalEncoder(), getVelocity).WillByDefault(ReturnPointee(&velocity));
    ON_CALL(motor, getTorque).WillByDefault(ReturnPointee(&torque));

    velocity = (maxVelocity - 1);
    torque = minTorque - 1;
    EXPECT_EQ(false, trigger.isTriggered());

    velocity = (-maxVelocity + 1);
    torque = -minTorque + 1;
    EXPECT_EQ(false, trigger.isTriggered());
}

TEST_F(MotorStallTriggerTest, torque_out_velocity_out_no_stall)
{
    float velocity;
    int16_t torque;

    ON_CALL(motor.getInternalEncoder(), isOnline).WillByDefault(Return(true));
    ON_CALL(motor.getInternalEncoder(), getVelocity).WillByDefault(ReturnPointee(&velocity));
    ON_CALL(motor, getTorque).WillByDefault(ReturnPointee(&torque));

    velocity = (maxVelocity + 1);
    torque = minTorque + 1;
    EXPECT_EQ(false, trigger.isTriggered());

    velocity = (-maxVelocity - 1);
    torque = -minTorque - 1;
    EXPECT_EQ(false, trigger.isTriggered());
}