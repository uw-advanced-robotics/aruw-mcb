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

#ifndef OLED_DISPLAY_MOCK_HPP_
#define OLED_DISPLAY_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/drivers.hpp"

#include "aruwsrc/display/oled_display.hpp"

namespace aruwsrc
{
namespace mock
{
class OledDisplayMock : public display::OledDisplay
{
public:
    explicit OledDisplayMock(
        tap::Drivers *drivers,
        aruwsrc::serial::VisionCoprocessor *vc,
        can::TurretMCBCanComm *turretMCBCanCommBus1,
        can::TurretMCBCanComm *turretMCBCanCommBus2,
        aruwsrc::virtualMCB::MCBLite *mcbLite1,
        aruwsrc::virtualMCB::MCBLite *mcbLite2);
    virtual ~OledDisplayMock();
    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(bool, updateDisplay, (), (override));
    MOCK_METHOD(void, updateMenu, (), (override));
};  // class OledDisplayMock
}  // namespace mock
}  // namespace aruwsrc

#endif  // OLED_DISPLAY_MOCK_HPP_
