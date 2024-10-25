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

using namespace testing;

using namespace aruwsrc::mock;
using namespace aruwsrc::can::capbank;
using namespace aruwsrc::control::capbank;

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

TEST_F(CapBankSubsystemTests, disabled_and_stop_called_in_safe_disconnect)
{
    CapBankSubsystem dut(&drivers, capBank);
    dut.enableCapacitors();

    EXPECT_CALL(capBank, stop);

    dut.refreshSafeDisconnect();

    EXPECT_FALSE(dut.enabled());
}

TEST_F(CapBankSubsystemTests, no_cap_update_when_message_timer_does_not_increase)
{
    CapBankSubsystem dut(&drivers, capBank);

    EXPECT_CALL(capBank, start).Times(0);
    EXPECT_CALL(capBank, stop).Times(0);
    EXPECT_CALL(capBank, ping).Times(0);

    dut.refresh();
    dut.refresh();
}

TEST_F(CapBankSubsystemTests, expect_ping_when_not_changing_state)
{
    CapBankSubsystem dut(&drivers, capBank);
    dut.disableCapacitors();
    capBank.state = State::DISABLED;

    EXPECT_CALL(capBank, start).Times(0);
    EXPECT_CALL(capBank, stop).Times(0);
    EXPECT_CALL(capBank, ping).Times(2);

    clock.time = 21;
    dut.refresh();
    clock.time = 42;
    dut.refresh();
}

TEST_F(CapBankSubsystemTests, expect_start_when_enabling)
{
    CapBankSubsystem dut(&drivers, capBank);
    dut.enableCapacitors();
    capBank.state = State::DISABLED;

    EXPECT_CALL(capBank, start).Times(1);
    EXPECT_CALL(capBank, stop).Times(0);
    EXPECT_CALL(capBank, ping).Times(0);

    clock.time = 21;
    dut.refresh();
}

TEST_F(CapBankSubsystemTests, expect_start_when_enabling_then_ping_when_changed)
{
    CapBankSubsystem dut(&drivers, capBank);
    dut.enableCapacitors();
    capBank.state = State::DISABLED;

    EXPECT_CALL(capBank, start).Times(1);
    EXPECT_CALL(capBank, stop).Times(0);
    EXPECT_CALL(capBank, ping).Times(0);

    clock.time = 21;
    dut.refresh();

    capBank.state = State::CHARGE_DISCHARGE;

    EXPECT_CALL(capBank, start).Times(0);
    EXPECT_CALL(capBank, stop).Times(0);
    EXPECT_CALL(capBank, ping).Times(1);
    clock.time = 42;
    dut.refresh();
}

TEST_F(CapBankSubsystemTests, expect_stop_when_disabling)
{
    CapBankSubsystem dut(&drivers, capBank);
    dut.disableCapacitors();
    capBank.state = State::CHARGE_DISCHARGE;

    EXPECT_CALL(capBank, start).Times(0);
    EXPECT_CALL(capBank, stop).Times(1);
    EXPECT_CALL(capBank, ping).Times(0);

    clock.time = 21;
    dut.refresh();
}

TEST_F(CapBankSubsystemTests, expect_stop_when_disabling_then_ping_when_changed)
{
    CapBankSubsystem dut(&drivers, capBank);
    dut.disableCapacitors();
    capBank.state = State::CHARGE_DISCHARGE;

    EXPECT_CALL(capBank, start).Times(0);
    EXPECT_CALL(capBank, stop).Times(1);
    EXPECT_CALL(capBank, ping).Times(0);

    clock.time = 21;
    dut.refresh();

    capBank.state = State::DISABLED;

    EXPECT_CALL(capBank, start).Times(0);
    EXPECT_CALL(capBank, stop).Times(0);
    EXPECT_CALL(capBank, ping).Times(1);
    clock.time = 42;
    dut.refresh();
}

TEST_F(CapBankSubsystemTests, expect_no_sprint_when_disabling)
{
    CapBankSubsystem dut(&drivers, capBank);
    dut.disableCapacitors();
    capBank.state = State::CHARGE_DISCHARGE;
    capBank.setSprinting(SprintMode::SPRINT);

    EXPECT_CALL(capBank, start).Times(0);
    EXPECT_CALL(capBank, stop).Times(1);
    EXPECT_CALL(capBank, ping).Times(0);

    clock.time = 21;
    dut.refresh();
    EXPECT_FALSE(capBank.isSprinting());
}