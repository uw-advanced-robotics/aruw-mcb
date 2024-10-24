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

#ifndef TURRET_SUBSYSTEM_MOCK_HPP_
#define TURRET_SUBSYSTEM_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/mock/motor_interface_mock.hpp"

#include "aruwsrc/control/turret/turret_subsystem.hpp"

namespace aruwsrc
{
namespace mock
{
class TurretSubsystemMock : public aruwsrc::control::turret::TurretSubsystem
{
public:
    TurretSubsystemMock(tap::Drivers *drivers);
    virtual ~TurretSubsystemMock();

    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, refresh, (), (override));
    MOCK_METHOD(const char *, getName, (), (const override));
    MOCK_METHOD(bool, isOnline, (), (const override));

private:
    static constexpr aruwsrc::control::turret::TurretMotorConfig MOTOR_CONFIG = {
        .startAngle = M_PI_2,
        .startEncoderValue = 0,
        .minAngle = 0,
        .maxAngle = M_PI,
        .limitMotorAngles = false,
    };

    testing::NiceMock<tap::mock::MotorInterfaceMock> m;
};  // class TurretSubsystemMock
}  // namespace mock
}  // namespace aruwsrc

#endif  // TURRET_SUBSYSTEM_MOCK_HPP_
