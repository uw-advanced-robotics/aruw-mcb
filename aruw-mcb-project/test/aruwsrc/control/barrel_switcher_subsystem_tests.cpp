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
    aruwsrc::control::StallThresholdConfig config{
        .maxRPM = 100,
        .minTorque = 10,
    };
    BarrelSwitcherSubsystem barrelSwitcher;
};

TEST_F(BarrelSwitcherSubsystemTest, stop_changes_state)
{
    barrelSwitcher.stop();
    EXPECT_EQ(BarrelState::IDLE, barrelSwitcher.getBarrelState());
}

TEST_F(BarrelSwitcherSubsystemTest, correctly_detects_stall) {
    int16_t rpm;
    int16_t torque;

    ON_CALL(barrelSwitcher.motor, getShaftRPM).WillByDefault(ReturnPointee(&rpm));
    ON_CALL(barrelSwitcher.motor, getTorque).WillByDefault(ReturnPointee(&torque));

    rpm = config.maxRPM + 1;
    torque = config.minTorque + 1;
    EXPECT_EQ(false, barrelSwitcher.isStalled());
    
    rpm = config.maxRPM - 1;
    torque = config.minTorque + 1;
    EXPECT_EQ(true, barrelSwitcher.isStalled());

    rpm = config.maxRPM + 1;
    torque = config.minTorque - 1;
    EXPECT_EQ(false, barrelSwitcher.isStalled());

    rpm = config.maxRPM - 1;
    torque = config.minTorque - 1;
    EXPECT_EQ(false, barrelSwitcher.isStalled());

    rpm = config.maxRPM + 1;
    torque = config.minTorque + 1;
    EXPECT_EQ(false, barrelSwitcher.isStalled());
    
    rpm = config.maxRPM - 1;
    torque = config.minTorque + 1;
    EXPECT_EQ(true, barrelSwitcher.isStalled());

    rpm = config.maxRPM + 1;
    torque = config.minTorque - 1;
    EXPECT_EQ(false, barrelSwitcher.isStalled());

    rpm = config.maxRPM - 1;
    torque = config.minTorque - 1;
    EXPECT_EQ(false, barrelSwitcher.isStalled());
}
