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

#include <gmock/gmock.h>

#include "aruwsrc/control/turret/turret_subsystem.hpp"

namespace aruwsrc
{
namespace mock
{
class TurretSubsystemMock : public aruwsrc::turret::TurretSubsystem
{
public:
    explicit TurretSubsystemMock(aruwlib::Drivers* drivers) : TurretSubsystem(drivers) {}

    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, refresh, (), (override));
    MOCK_METHOD(bool, isTurretOnline, (), (const override));
    MOCK_METHOD(int32_t, getYawVelocity, (), (const override));
    MOCK_METHOD(int32_t, getPitchVelocity, (), (const override));
    MOCK_METHOD(float, getYawAngleFromCenter, (), (const override));
    MOCK_METHOD(float, getPitchAngleFromCenter, (), (const override));
    MOCK_METHOD(const aruwlib::algorithms::ContiguousFloat&, getYawAngle, (), (const override));
    MOCK_METHOD(const aruwlib::algorithms::ContiguousFloat&, getPitchAngle, (), (const override));
    MOCK_METHOD(void, setYawMotorOutput, (float out), (override));
    MOCK_METHOD(void, setPitchMotorOutput, (float out), (override));
    MOCK_METHOD(float, yawFeedForwardCalculation, (float desiredChassisRotation), (override));
    MOCK_METHOD(void, setYawTarget, (float target), (override));
    MOCK_METHOD(void, setPitchTarget, (float target), (override));
    MOCK_METHOD(float, getYawTarget, (), (const override));
    MOCK_METHOD(float, getPitchTarget, (), (const override));
    MOCK_METHOD(void, updateCurrentTurretAngles, (), (override));
    MOCK_METHOD(void, runHardwareTests, (), (override));
    MOCK_METHOD(const char*, getName, (), (override));
};  // class TowSubsystem
}  // namespace mock
}  // namespace aruwsrc
