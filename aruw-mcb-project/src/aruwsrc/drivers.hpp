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

#ifndef DRIVERS_HPP_
#define DRIVERS_HPP_

#include "tap/drivers.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "aruwsrc/mock/legacy_vision_coprocessor_mock.hpp"
#include "aruwsrc/mock/oled_display_mock.hpp"
#include "aruwsrc/mock/turret_mcb_can_comm_mock.hpp"
#else
#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/communication/serial/legacy_vision_coprocessor.hpp"
#include "aruwsrc/display/oled_display.hpp"
#endif

namespace aruwsrc
{
class Drivers : public tap::Drivers
{
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
public:
#endif
    Drivers()
        : tap::Drivers(),
          legacyVisionCoprocessor(this),
          oledDisplay(this),
          turretMCBCanComm(this)
    {
    }

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    testing::NiceMock<mock::LegacyVisionCoprocessorMock> legacyVisionCoprocessor;
    testing::NiceMock<mock::OledDisplayMock> oledDisplay;
    testing::NiceMock<mock::TurretMCBCanCommMock> turretMCBCanComm;
#else
public:
    serial::LegacyVisionCoprocessor legacyVisionCoprocessor;
    display::OledDisplay oledDisplay;
    can::TurretMCBCanComm turretMCBCanComm;
#endif
};  // class aruwsrc::Drivers
}  // namespace aruwsrc

#endif  // DRIVERS_HPP_
