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

#ifndef LEGACY_VISION_COPROCESSOR_MOCK_HPP_
#define LEGACY_VISION_COPROCESSOR_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/communication/serial/legacy_vision_coprocessor.hpp"

namespace aruwsrc
{
namespace mock
{
class LegacyVisionCoprocessorMock : public serial::LegacyVisionCoprocessor
{
public:
    LegacyVisionCoprocessorMock(aruwsrc::Drivers *drivers);
    virtual ~LegacyVisionCoprocessorMock();

    MOCK_METHOD(void, initializeCV, (), (override));
    MOCK_METHOD(void, messageReceiveCallback, (const SerialMessage &), (override));
    MOCK_METHOD(bool, sendMessage, (), (override));
    MOCK_METHOD(void, beginAutoAim, (), (override));
    MOCK_METHOD(void, stopAutoAim, (), (override));
    MOCK_METHOD(const TurretAimData &, getLastAimData, (), (const override));
    MOCK_METHOD(bool, lastAimDataValid, (), (const override));
    MOCK_METHOD(void, attachTurret, (aruwsrc::control::turret::TurretSubsystemInterface *), (override));
    MOCK_METHOD(
        void,
        attachChassis,
        (tap::control::chassis::ChassisSubsystemInterface *),
        (override));
};  // class LegacyVisionCoprocessorMock
}  // namespace mock
}  // namespace aruwsrc

#endif  // LEGACY_VISION_COPROCESSOR_MOCK_HPP_
