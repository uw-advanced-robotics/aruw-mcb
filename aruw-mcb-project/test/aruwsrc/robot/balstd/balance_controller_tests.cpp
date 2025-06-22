/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/mock/balstd/balstd_control_operator_interface_mock.hpp"
#include "aruwsrc/robot/balstd/chassis/balstd_chassis_constants.hpp"
#include "aruwsrc/robot/balstd/chassis/controllers/balance_controller.hpp"

using namespace testing;
using namespace aruwsrc::mock;
using namespace aruwsrc::balstd::chassis;
using namespace aruwsrc::balstd::chassis::controllers;

// BalanceController::Config CONFIG {

// }

class BalanceControllerTest : public Test
{
protected:
    BalanceControllerTest()
        : controlOperatorInterface(&drivers),
          controller(controlOperatorInterface, BALANCE_CONTROLLER_CONFIG)
    {
    }

    tap::Drivers drivers;
    NiceMock<BalstdControlOperatorInterfaceMock> controlOperatorInterface;
    BalanceController controller;
};

TEST_F(BalanceControllerTest, lqr_gain_scheduling)
{
    // Compares gains generated from polynomial fit to gains directly computed for 0.17m leg length
    CMSISMat<2, 6> gains = controller.getLQRGains(0.17);

    CMSISMat<2, 6> expected(
        {-37.831,
         -5.031,
         -20.811,
         -17.182,
         31.92,
         5.0632,
         12.604,
         1.6434,
         8.1786,
         6.2358,
         68.298,
         6.4245});

    for (int i = 0; i < 12; i++)
    {
        EXPECT_NEAR(gains.data[i], expected.data[i], 1e-2);
    }
}
