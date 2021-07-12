/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifdef TARGET_SENTINEL
#ifndef TOW_SUBSYSTEM_MOCK_HPP_
#define TOW_SUBSYSTEM_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/control/engineer/tow_subsystem.hpp"

namespace aruwsrc
{
namespace mock
{
class TowSubsystemMock : public aruwsrc::engineer::TowSubsystem
{
public:
    TowSubsystemMock(
        aruwlib::Drivers *drivers,
        aruwlib::gpio::Digital::OutputPin leftTowPin,
        aruwlib::gpio::Digital::OutputPin rightTowPin,
        aruwlib::gpio::Digital::InputPin leftTowLimitSwitchPin,
        aruwlib::gpio::Digital::InputPin rightTowLimitSwitchPin);
    virtual ~TowSubsystemMock();

    MOCK_METHOD(void, setLeftClamped, (bool isClamped), (override));
    MOCK_METHOD(bool, getLeftClamped, (), (const override));
    MOCK_METHOD(void, setRightClamped, (bool isClamped), (override));
    MOCK_METHOD(bool, getRightClamped, (), (const override));
    MOCK_METHOD(bool, getLeftLimitSwitchTriggered, (), (const override));
    MOCK_METHOD(bool, getRightLeftLimitSwitchTriggered, (), (const override));
};  // class TowSubsystem
}  // namespace mock
}  // namespace aruwsrc

#endif  // TOW_SUBSYSTEM_MOCK_HPP_
#endif
