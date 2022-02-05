/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef VISION_COPROCESSOR_MOCK_HPP_
#define VISION_COPROCESSOR_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"

namespace aruwsrc
{
namespace mock
{
class VisionCoprocessorMock : public serial::VisionCoprocessor
{
public:
    VisionCoprocessorMock(aruwsrc::Drivers *drivers);
    virtual ~VisionCoprocessorMock();

    MOCK_METHOD(void, initializeCV, (), (override));
    MOCK_METHOD(void, messageReceiveCallback, (const SerialMessage &), (override));
    MOCK_METHOD(void, sendMessage, (), (override));
    MOCK_METHOD(const TurretAimData &, getLastAimData, (), (const override));
};  // class VisionCoprocessorMock
}  // namespace mock
}  // namespace aruwsrc

#endif  // LEGACY_VISION_COPROCESSOR_MOCK_HPP_
