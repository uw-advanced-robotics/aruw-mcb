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

#ifndef YAW_TURRET_SUBSYSTEM_MOCK_HPP_
#define YAW_TURRET_SUBSYSTEM_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/mock/motor_interface_mock.hpp"

#include "aruwsrc/control/turret/yaw_turret_subsystem.hpp"

namespace aruwsrc 
{
namespace mock
{
class YawTurretSubsystemMock : public aruwsrc::control::turret::YawTurretSubsystem
{
public:
    YawTurretSubsystemMock(tap::Drivers *drivers);
    ~YawTurretSubsystemMock();

    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, refresh, (), (override));
//    MOCK_METHOD(const char *, getName, (), (override));
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
};  // class YawTurretSubsystemMock
}  // namespace mock
}  // namespace aruwsrc

#endif  // YAW_TURRET_SUBSYSTEM_MOCK_HPP_
