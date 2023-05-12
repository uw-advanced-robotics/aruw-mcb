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
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/barrel-switcher/barrel_switcher_subsystem.hpp"

using namespace aruwsrc::control;
using namespace testing;

class BarrelSwitcherSubsystemTest : public Test
{
protected:
    BarrelSwitcherSubsystemTest() : barrelSwitcher(&drivers, config, motorid) {}

    tap::Drivers drivers;
    tap::motor::MotorId motorid;
    aruwsrc::control::HomingConfig config = aruwsrc::control::HomingConfig{
        .minRPM = -100,
        .maxRPM = 100,
        .minTorque = -10,
        .maxTorque = 10
    };
    BarrelSwitcherSubsystem barrelSwitcher;
};


TEST_F(BarrelSwitcherSubsystemTest, move_to_lower_bound_changes_state) {
    barrelSwitcher.moveTowardLowerBound();
    EXPECT_EQ(BarrelState::MOVING_TOWARD_LOWER_BOUND, barrelSwitcher.getBarrelState());
}

TEST_F(BarrelSwitcherSubsystemTest, move_to_upper_bound_changes_state)
{
    barrelSwitcher.moveTowardUpperBound();
    EXPECT_EQ(BarrelState::MOVING_TOWARD_UPPER_BOUND, barrelSwitcher.getBarrelState());
}

TEST_F(BarrelSwitcherSubsystemTest, stop_changes_state)
{
    barrelSwitcher.stop();
    EXPECT_EQ(BarrelState::USING_RIGHT_BARREL, barrelSwitcher.getBarrelState());
}

TEST_F(BarrelSwitcherSubsystemTest, correctly_detects_stall) {
    int16_t rpm;
    int16_t torque;

    ON_CALL(barrelSwitcher.motor, getShaftRPM).WillByDefault(ReturnPointee(&rpm));
    ON_CALL(barrelSwitcher.motor, getTorque).WillByDefault(ReturnPointee(&torque));

    rpm = config.maxRPM + 1;
    torque = config.maxTorque + 1;
    EXPECT_EQ(false, barrelSwitcher.isStalled());
    
    rpm = config.maxRPM - 1;
    torque = config.maxTorque + 1;
    EXPECT_EQ(true, barrelSwitcher.isStalled());

    rpm = config.maxRPM + 1;
    torque = config.maxTorque - 1;
    EXPECT_EQ(false, barrelSwitcher.isStalled());

    rpm = config.maxRPM - 1;
    torque = config.maxTorque - 1;
    EXPECT_EQ(false, barrelSwitcher.isStalled());


    rpm = config.minRPM + 1;
    torque = config.minTorque + 1;
    EXPECT_EQ(false, barrelSwitcher.isStalled());
    
    rpm = config.minRPM - 1;
    torque = config.minTorque + 1;
    EXPECT_EQ(false, barrelSwitcher.isStalled());

    rpm = config.minRPM + 1;
    torque = config.minTorque - 1;
    EXPECT_EQ(true, barrelSwitcher.isStalled());

    rpm = config.minRPM - 1;
    torque = config.minTorque - 1;
    EXPECT_EQ(false, barrelSwitcher.isStalled());
}

TEST_F(BarrelSwitcherSubsystemTest, isBetweenPositions_correct)
{
    int16_t encoderPosition;
    

    ON_CALL(barrelSwitcher.motor, getEncoderUnwrapped).WillByDefault(ReturnPointee(&encoderPosition));

    //set upper bound
    encoderPosition = 1000;
    barrelSwitcher.setUpperBound();

    encoderPosition = 0;
    EXPECT_FALSE(barrelSwitcher.isBetweenPositions());

    encoderPosition = 1000;
    EXPECT_FALSE(barrelSwitcher.isBetweenPositions());

    encoderPosition = aruwsrc::control::MOTOR_POSITION_TOLERANCE;
    EXPECT_FALSE(barrelSwitcher.isBetweenPositions());

    encoderPosition = 1000 - aruwsrc::control::MOTOR_POSITION_TOLERANCE;
    EXPECT_FALSE(barrelSwitcher.isBetweenPositions());

    encoderPosition = 500;
    EXPECT_TRUE(barrelSwitcher.isBetweenPositions());
}