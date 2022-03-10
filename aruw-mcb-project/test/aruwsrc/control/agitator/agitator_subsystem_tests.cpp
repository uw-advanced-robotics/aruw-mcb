/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/control/agitator/agitator_subsystem.hpp"
#include "aruwsrc/drivers.hpp"

using namespace testing;
using namespace aruwsrc::agitator;
using namespace aruwsrc;
using namespace tap::motor;

static constexpr float GEAR_RATIO = AgitatorSubsystem::AGITATOR_GEAR_RATIO_GM3508;
static constexpr float ENC_TO_ANGLE_RATIO =
    2.0 * M_PI / (static_cast<float>(DjiMotor::ENC_RESOLUTION) * GEAR_RATIO);

class AgitatorSubsystemTest : public TestWithParam<std::tuple<float, uint32_t, bool>>
{
protected:
    AgitatorSubsystemTest()
        : agitator(
              &drivers,
              {.kp = 1'000, .maxOutput = 1},
              GEAR_RATIO,
              tap::motor::MOTOR1,
              tap::can::CanBus::CAN_BUS1,
              false,
              std::get<0>(GetParam()),
              std::get<1>(GetParam()),
              std::get<2>(GetParam()))
    {
    }

    void SetUp() override
    {
        ON_CALL(agitator.agitatorMotor, getEncoderUnwrapped)
            .WillByDefault(ReturnPointee(&encUnwrapped));
        ON_CALL(agitator.agitatorMotor, isMotorOnline).WillByDefault(ReturnPointee(&motorOnline));
        ON_CALL(agitator.agitatorMotor, getShaftRPM).WillByDefault(Return(0));
    }

    tap::arch::clock::ClockStub clock;
    Drivers drivers;
    AgitatorSubsystem agitator;

    int32_t encUnwrapped = 0;
    bool motorOnline = true;
};

TEST_P(AgitatorSubsystemTest, initialize_initializes_motor)
{
    EXPECT_CALL(agitator.agitatorMotor, initialize);

    agitator.initialize();
}

TEST_P(AgitatorSubsystemTest, getSetpoint_returns_setSetpoint)
{
    agitator.setSetpoint(100);
    EXPECT_EQ(100, agitator.getSetpoint());

    agitator.setSetpoint(-100);
    EXPECT_EQ(-100, agitator.getSetpoint());
}

TEST_P(AgitatorSubsystemTest, refresh_runs_pid_controller)
{
    static constexpr float UPDATE_INCR = M_PI / 100;

    ON_CALL(agitator.agitatorMotor, setDesiredOutput).WillByDefault([&](int32_t out) {
        encUnwrapped += out < 0 ? -UPDATE_INCR / ENC_TO_ANGLE_RATIO
                                : out > 0 ? UPDATE_INCR / ENC_TO_ANGLE_RATIO : 0;
    });

    agitator.calibrateHere();

    agitator.setSetpoint(M_PI);

    for (size_t i = 0; i < 200; i++)
    {
        agitator.refresh();
    }

    EXPECT_NEAR(M_PI, agitator.getCurrentValue(), UPDATE_INCR);
}

TEST_P(AgitatorSubsystemTest, getCurrentValue_returns_0_when_not_calibrated)
{
    EXPECT_NEAR(0.0f, agitator.getCurrentValue(), 1E-3);
}

TEST_P(AgitatorSubsystemTest, getCurrentValue_proportional_to_wrapped_encoder)
{
    // must first initialize the agitator
    agitator.calibrateHere();

    std::vector<int32_t> valuesToTry{-100, 0, 100, -100'000, 100'000};

    for (int32_t i : valuesToTry)
    {
        encUnwrapped = i;
        EXPECT_NEAR(ENC_TO_ANGLE_RATIO * encUnwrapped, agitator.getCurrentValue(), 1E-3);
    }
}

TEST_P(AgitatorSubsystemTest, calibrateHere_resets_value_to_0)
{
    std::vector<int32_t> valuesToTry{-100, 0, 100, -100'000, 100'000};

    for (int32_t i : valuesToTry)
    {
        encUnwrapped = i;
        agitator.calibrateHere();
        EXPECT_NEAR(0.0f, agitator.getCurrentValue(), 1E-3);
    }
}

TEST_P(AgitatorSubsystemTest, getJamSetpointTolerance_returns_setpoint_tolerance)
{
    EXPECT_EQ(std::get<0>(GetParam()), agitator.getJamSetpointTolerance());
}

TEST_P(AgitatorSubsystemTest, isJammed_false_when_within_tolerance)
{
    std::vector<int32_t> valuesToTry{-100, 0, 100, -100'000, 100'000};

    agitator.calibrateHere();

    for (int32_t i : valuesToTry)
    {
        clock.time += 1000;
        agitator.setSetpoint(ENC_TO_ANGLE_RATIO * i);
        encUnwrapped = i;
        agitator.refresh();
    }

    EXPECT_FALSE(agitator.isJammed());
}

TEST_P(AgitatorSubsystemTest, isJammed_true_when_not_within_tolerance)
{
    agitator.calibrateHere();

    agitator.setSetpoint(std::get<0>(GetParam()) + 1);

    clock.time += std::get<1>(GetParam()) + 1;
    agitator.refresh();

    // if jam logic enabled, should be jammed, otherwise no
    EXPECT_EQ(std::get<2>(GetParam()), agitator.isJammed());
}

TEST_P(AgitatorSubsystemTest, isJammed_false_when_turned_off_then_on)
{
    agitator.calibrateHere();

    agitator.setSetpoint(std::get<0>(GetParam()) + 1);

    clock.time += std::get<1>(GetParam()) + 1;
    agitator.refresh();

    // will be jammed
    EXPECT_EQ(std::get<2>(GetParam()), agitator.isJammed());

    motorOnline = false;

    agitator.refresh();

    motorOnline = true;

    agitator.refresh();

    // won't be jammed now that motor is back online
    EXPECT_FALSE(agitator.isJammed());
}

TEST_P(AgitatorSubsystemTest, isJammed_false_after_jam_cleared)
{
    agitator.calibrateHere();

    agitator.setSetpoint(std::get<0>(GetParam()) + 1);

    clock.time += std::get<1>(GetParam()) + 1;
    agitator.refresh();

    agitator.clearJam();

    EXPECT_FALSE(agitator.isJammed());
}

TEST_P(AgitatorSubsystemTest, isOnline_same_as_agitator_motor_online)
{
    EXPECT_TRUE(agitator.isOnline());
    motorOnline = false;
    EXPECT_FALSE(agitator.isOnline());
}

INSTANTIATE_TEST_SUITE_P(
    AgitatorSubsystem,
    AgitatorSubsystemTest,
    Values(
        std::tuple<float, uint32_t, bool>(10, 10, true),
        std::tuple<float, uint32_t, bool>(10, 10, false)));
