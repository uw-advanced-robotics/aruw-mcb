/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include <cmath>
#include <deque>

#include <gtest/gtest.h>

#include "aruwsrc/control/autotune/chassis_wheel_pid_autotune.hpp"

using namespace aruwsrc::control::autotune;

TEST(WheelRelayTest, flips_only_outside_hysteresis)
{
    WheelRelayTest test({.amplitude = 100, .hysteresis = 2});

    EXPECT_EQ(100, test.update(5.0f, 0, 0));
    EXPECT_EQ(100, test.update(-1.9f, 0, 1));
    EXPECT_EQ(-100, test.update(-2.1f, 0, 2));
    EXPECT_EQ(-100, test.update(1.9f, 0, 3));
    EXPECT_EQ(100, test.update(2.1f, 0, 4));
}

// Simulated wheel: integrator with friction and a few loops of delay.
TEST(WheelRelayTest, closed_loop_wheel_finds_bias_and_oscillates)
{
    constexpr float SETPOINT = 120.0f;
    constexpr float RELAY = 1500.0f;
    constexpr float FRICTION = 500.0f;
    constexpr float RPM_PER_S_PER_OUTPUT = 1000.0f / RELAY;
    constexpr uint32_t DT_US = 2000;
    constexpr size_t DELAY_STEPS = 5;
    constexpr float HYSTERESIS = 2.0f;

    WheelRelayTest test({.amplitude = RELAY, .hysteresis = HYSTERESIS});

    float rpm = 0;
    std::deque<float> pending(DELAY_STEPS, 0.0f);
    uint32_t timeUs = 0;
    for (int i = 0; i < 100'000 && !test.isDone(); i++)
    {
        pending.push_back(test.update(SETPOINT - rpm, rpm, timeUs));
        const float friction = rpm > 0 ? FRICTION : 0.0f;
        rpm += (pending.front() - friction) * RPM_PER_S_PER_OUTPUT * DT_US / 1e6f;
        pending.pop_front();
        timeUs += DT_US;
    }

    ASSERT_TRUE(test.isDone());
    EXPECT_NEAR(FRICTION, test.getBias(), 0.05f * FRICTION);

    const float amplitude = test.getOscillationAmplitude();
    EXPECT_GT(amplitude, HYSTERESIS);
    EXPECT_NEAR(4.0f * amplitude / 1000.0f, test.getPeriodSeconds(), 2.0f * DT_US / 1e6f);
    EXPECT_NEAR(
        4.0f * RELAY / (M_PI * std::sqrt(amplitude * amplitude - HYSTERESIS * HYSTERESIS)),
        test.getUltimateGain(),
        1e-2);
}

TEST(WheelRelayTest, ultimate_gain_zero_before_measuring)
{
    WheelRelayTest test({.amplitude = 100, .hysteresis = 2});
    EXPECT_FALSE(test.isDone());
    EXPECT_EQ(0.0f, test.getUltimateGain());
    EXPECT_EQ(0.0f, test.getPeriodSeconds());
}

TEST(TyreusLuybenGains, matches_formula)
{
    PidGains gains = tyreusLuybenGains(10.0f, 0.1f);
    EXPECT_NEAR(4.5f, gains.kp, 1e-4);
    EXPECT_NEAR(4.5f / 0.22f, gains.ki, 1e-2);
    EXPECT_NEAR(4.5f * 0.1f / 6.3f, gains.kd, 1e-5);
}

TEST(TyreusLuybenGains, zero_for_invalid_input)
{
    EXPECT_EQ(0.0f, tyreusLuybenGains(0.0f, 0.1f).kp);
    EXPECT_EQ(0.0f, tyreusLuybenGains(10.0f, 0.0f).kp);
}
