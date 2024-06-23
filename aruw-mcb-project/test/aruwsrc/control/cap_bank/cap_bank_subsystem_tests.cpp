/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/communication/can/capacitor_bank.hpp"
#include "aruwsrc/control/cap_bank/cap_bank_sprint_command.hpp"
#include "aruwsrc/control/cap_bank/cap_bank_subsystem.hpp"
#include "aruwsrc/control/cap_bank/cap_bank_toggle_command.hpp"

#include "aruwsrc/mock/capacitor_bank_mock.hpp"

using namespace aruwsrc::mock;
using namespace aruwsrc::can::capbank;
using namespace aruwsrc::control::capbank;

TEST(CapBankSubsystem, constructor_sets_sprint_to_no_sprint)
{
    tap::Drivers drivers;
    testing::NiceMock<CapacitorBankMock> capBank(&drivers, tap::can::CanBus::CAN_BUS1, 1.0);

    capBank.setSprinting(SprintMode::SPRINT); // Make sure that the caps are in the WRONG state to verify its set
    CapBankSubsystem dut(&drivers, capBank);

    EXPECT_FALSE(capBank.isSprinting());
}

