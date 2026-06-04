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

TEST_F(CapBankSubsystemTests, safe_disconnect_requests_discharge_and_disables)
{
    CapBankSubsystem dut(&drivers, capBank);
    dut.enableCapacitors();

    EXPECT_CALL(capBank, setMode(Mode::SAFETY_DISCHARGE));

    dut.refreshSafeDisconnect();

    EXPECT_FALSE(dut.enabled());
}

TEST_F(CapBankSubsystemTests, no_cap_message_when_message_timer_does_not_increase)
{
    CapBankSubsystem dut(&drivers, capBank);

    EXPECT_CALL(capBank, setMode(_)).Times(0);

    dut.refresh();
    dut.refresh();
}

TEST_F(CapBankSubsystemTests, commands_standby_when_disabled)
{
    CapBankSubsystem dut(&drivers, capBank);
    dut.disableCapacitors();
    capBank.mode = Mode::STANDBY;

    EXPECT_CALL(capBank, setMode(Mode::STANDBY)).Times(2);

    clock.time = 21;
    dut.refresh();
    clock.time = 42;
    dut.refresh();
}

TEST_F(CapBankSubsystemTests, commands_charge_only_when_enabled_and_not_sprinting)
{
    CapBankSubsystem dut(&drivers, capBank);
    dut.enableCapacitors();
    capBank.setSprinting(SprintMode::NO_SPRINT);
    capBank.mode = Mode::STANDBY;

    EXPECT_CALL(capBank, setMode(Mode::CHARGE_ONLY)).Times(1);

    clock.time = 21;
    dut.refresh();
}

TEST_F(CapBankSubsystemTests, commands_boost_when_enabled_and_sprinting)
{
    CapBankSubsystem dut(&drivers, capBank);
    dut.enableCapacitors();
    capBank.setSprinting(SprintMode::SPRINT);
    capBank.mode = Mode::CHARGE_ONLY;

    EXPECT_CALL(capBank, setMode(Mode::BOOST)).Times(1);

    clock.time = 21;
    dut.refresh();
}

TEST_F(CapBankSubsystemTests, safety_discharge_latches_until_bank_returns_to_standby)
{
    CapBankSubsystem dut(&drivers, capBank);
    capBank.mode = Mode::BOOST;  // bank currently active

    dut.refreshSafeDisconnect();  // requests a safety discharge (commands it once directly)

    // While the bank has not yet reached STANDBY, refresh keeps commanding the discharge.
    EXPECT_CALL(capBank, setMode(Mode::SAFETY_DISCHARGE)).Times(1);
    clock.time = 21;
    dut.refresh();
    Mock::VerifyAndClearExpectations(&capBank);

    // Once the bank reports STANDBY the latch clears; disabled => STANDBY is commanded.
    capBank.mode = Mode::STANDBY;
    EXPECT_CALL(capBank, setMode(Mode::STANDBY)).Times(1);
    clock.time = 42;
    dut.refresh();
}

TEST_F(CapBankSubsystemTests, sprint_is_reset_when_disabled)
{
    CapBankSubsystem dut(&drivers, capBank);
    dut.disableCapacitors();
    capBank.mode = Mode::STANDBY;
    capBank.setSprinting(SprintMode::SPRINT);

    EXPECT_CALL(capBank, setMode(Mode::STANDBY)).Times(1);

    clock.time = 21;
    dut.refresh();
    EXPECT_FALSE(capBank.isSprinting());
}