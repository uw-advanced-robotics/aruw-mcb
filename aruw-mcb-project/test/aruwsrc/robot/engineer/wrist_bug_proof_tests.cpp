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

#include <cmath>

#include <gtest/gtest.h>

#include "tap/drivers.hpp"
#include "tap/mock/encoder_interface_mock.hpp"
#include "tap/mock/motor_interface_mock.hpp"

#include "aruwsrc/robot/engineer/engineer_wrist_constants.hpp"
#include "aruwsrc/robot/engineer/wrist/wrist_subsystem.hpp"

using namespace testing;
using namespace tap::algorithms::transforms;
using aruwsrc::engineer::wrist::WristSubsystem;

static constexpr float EPS = 1e-3f;

// only uses the public API that exists before and after the fix, so it runs on both branches
class WristBugProof : public Test
{
protected:
    tap::Drivers drivers;
    NiceMock<tap::mock::MotorInterfaceMock> m1, m2, m3;
    NiceMock<tap::mock::EncoderInterfaceMock> enc;
    WristSubsystem wrist{&drivers, m1, m2, m3, enc, aruwsrc::engineer::WRIST_CONFIG};

    Orientation commanded()
    {
        return WristSubsystem::getHypotheticalOrientation(
            wrist.getSetpointTheta1(),
            wrist.getSetpointTheta2(),
            wrist.getSetpointTheta3());
    }

    static void expectSameOrientation(const Orientation& actual, const Orientation& expected)
    {
        for (int i = 0; i < 9; i++)
        {
            EXPECT_NEAR(actual.matrix().data[i], expected.matrix().data[i], EPS) << "index " << i;
        }
    }
};

// Problem A: wrist exactly straight, cup rolled 0.5 rad. Old code gets theta1 and theta3 from
// atan2(0, 0), so the roll is lost and the wrist faces the wrong way.
TEST_F(WristBugProof, straight_wrist_keeps_the_requested_roll)
{
    Orientation target = WristSubsystem::getHypotheticalOrientation(0.3f, 0.0f, 0.2f);

    wrist.setSetpointOrientation(target);

    expectSameOrientation(commanded(), target);
}

// Problem B: wrist bent +0.1 rad, target bent -0.1 rad. Old code can only return positive
// theta2, so it flips theta1 and theta3 by 180 deg.
TEST_F(WristBugProof, crossing_straight_does_not_flip_theta1)
{
    wrist.setSetpointOrientation(WristSubsystem::getHypotheticalOrientation(0.2f, 0.1f, 0.1f));

    wrist.setSetpointOrientation(WristSubsystem::getHypotheticalOrientation(0.2f, -0.1f, 0.1f));

    EXPECT_NEAR(tap::algorithms::Angle(wrist.getSetpointTheta1()).minDifference(0.2f), 0, EPS);
    EXPECT_NEAR(wrist.getSetpointTheta2(), -0.1f, EPS);
}

// Problem C: float error pushes the matrix's first entry just past 1, so acosf returns NaN.
// setSetpointTheta2's limitValue turns that NaN into a limit, so a straight wrist gets told to
// bend 90 deg.
TEST_F(WristBugProof, float_error_keeps_straight_wrist_straight)
{
    Orientation target(tap::algorithms::CMSISMat<3, 3>(
        {1.0000001f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}));

    wrist.setSetpointOrientation(target);

    EXPECT_FALSE(std::isnan(wrist.getSetpointTheta1()));
    EXPECT_NEAR(wrist.getSetpointTheta2(), 0.0f, EPS);
    EXPECT_FALSE(std::isnan(wrist.getSetpointTheta3()));
}
