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

#ifndef CAPACITOR_BANK_MOCK_HPP_
#define CAPACITOR_BANK_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/communication/can/capacitor_bank.hpp"

namespace aruwsrc::mock
{
namespace
{
using namespace aruwsrc::can::capbank;
}

class CapacitorBankMock : public CapacitorBank
{
public:
    CapacitorBankMock(tap::Drivers* drivers, tap::can::CanBus canBus, const float capacitance);
    virtual ~CapacitorBankMock();

    MOCK_METHOD(
        void,
        initialize,
        (),
        (override));

    MOCK_METHOD(
        void,
        start,
        (),
        (const override));

    MOCK_METHOD(
        void,
        stop,
        (),
        (const override));

    MOCK_METHOD(
        void,
        ping,
        (),
        (const override));

    MOCK_METHOD(
        void,
        setPowerLimit,
        (uint16_t watts),
        (override));
};  // class CapacitorBankMock
}  // namespace aruwsrc::mock

#endif  // CAPACITOR_BANK_MOCK_HPP_
