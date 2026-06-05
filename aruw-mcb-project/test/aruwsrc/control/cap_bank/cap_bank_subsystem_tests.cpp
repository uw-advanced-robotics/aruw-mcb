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

#include "aruwsrc/communication/can/cap-bank/capacitor_bank.hpp"
#include "aruwsrc/control/cap-bank/cap_bank_sprint_command.hpp"
#include "aruwsrc/control/cap-bank/cap_bank_subsystem.hpp"
#include "aruwsrc/control/cap-bank/cap_bank_toggle_command.hpp"
#include "aruwsrc/mock/capacitor_bank_mock.hpp"

using namespace testing;

using namespace aruwsrc::mock;
using namespace aruwsrc::communication::can::cap_bank;
using namespace aruwsrc::control::cap_bank;

class CapBankSubsystemTests : public Test
{
public:
    CapBankSubsystemTests() : drivers(), capBank(&drivers, tap::can::CanBus::CAN_BUS1, 1.0), clock()
    {
    }

    tap::Drivers drivers;
    testing::NiceMock<CapacitorBankMock> capBank;
    tap::arch::clock::ClockStub clock;
};

TEST_F(CapBankSubsystemTests, constructor_sets_sprint_to_no_sprint)
{
    // Make sure that the caps are in the WRONG state to verify its set
    capBank.setSprinting(SprintMode::SPRINT);

    CapBankSubsystem dut(&drivers, capBank);

    EXPECT_FALSE(capBank.isSprinting());
}

TEST_F(CapBankSubsystemTests, safe_disconnect_commands_off_and_disables)
{
    CapBankSubsystem dut(&drivers, capBank);
    dut.enableCapacitors();

    EXPECT_CALL(capBank, sendCascadeCommand(CapCommandMode::OFF));

    dut.refreshSafeDisconnect();

    EXPECT_FALSE(dut.enabled());
}

TEST_F(CapBankSubsystemTests, no_command_when_message_timer_does_not_increase)
{
    CapBankSubsystem dut(&drivers, capBank);

    EXPECT_CALL(capBank, sendCascadeCommand(_)).Times(0);

    dut.refresh();
    dut.refresh();
}

TEST_F(CapBankSubsystemTests, commands_off_when_disabled)
{
    CapBankSubsystem dut(&drivers, capBank);
    dut.disableCapacitors();

    EXPECT_CALL(capBank, sendCascadeCommand(CapCommandMode::OFF)).Times(2);

    clock.time = 21;
    dut.refresh();
    clock.time = 42;
    dut.refresh();
}

TEST_F(CapBankSubsystemTests, commands_charge_when_enabled_and_not_sprinting)
{
    CapBankSubsystem dut(&drivers, capBank);
    dut.enableCapacitors();
    capBank.setSprinting(SprintMode::NO_SPRINT);

    EXPECT_CALL(capBank, sendCascadeCommand(CapCommandMode::CHARGE)).Times(1);

    clock.time = 21;
    dut.refresh();
}

TEST_F(CapBankSubsystemTests, commands_discharge_when_enabled_and_sprinting)
{
    CapBankSubsystem dut(&drivers, capBank);
    dut.enableCapacitors();
    capBank.setSprinting(SprintMode::SPRINT);

    EXPECT_CALL(capBank, sendCascadeCommand(CapCommandMode::DISCHARGE)).Times(1);

    clock.time = 21;
    dut.refresh();
}

TEST_F(CapBankSubsystemTests, sprint_is_reset_when_disabled)
{
    CapBankSubsystem dut(&drivers, capBank);
    dut.disableCapacitors();
    capBank.setSprinting(SprintMode::SPRINT);

    EXPECT_CALL(capBank, sendCascadeCommand(CapCommandMode::OFF)).Times(1);

    clock.time = 21;
    dut.refresh();
    EXPECT_FALSE(capBank.isSprinting());
}
