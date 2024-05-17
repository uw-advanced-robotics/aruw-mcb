/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ROBOT_TURRET_SUBSYSTEM_MOCK_HPP_
#define ROBOT_TURRET_SUBSYSTEM_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/mock/motor_interface_mock.hpp"

#include "aruwsrc/control/turret/robot_turret_subsystem.hpp"

namespace aruwsrc
{
namespace mock
{
class RobotTurretSubsystemMock : public aruwsrc::control::turret::RobotTurretSubsystem
{
public:
    RobotTurretSubsystemMock(tap::Drivers *drivers);
    virtual ~RobotTurretSubsystemMock();

    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, refresh, (), (override));
    MOCK_METHOD(const char *, getName, (), (const override));
    MOCK_METHOD(void, onHardwareTestStart, (), (override));
    MOCK_METHOD(bool, isOnline, (), (const override));
    MOCK_METHOD(modm::Vector3f, getTurretOffset, (), (const override));
    MOCK_METHOD(float, getPitchOffset, (), (const override));
    MOCK_METHOD(float, getWorldYaw, (), (const override));
    MOCK_METHOD(float, getWorldPitch, (), (const override));
    MOCK_METHOD(uint32_t, getLastMeasurementTimeMicros, (), (const override));

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
