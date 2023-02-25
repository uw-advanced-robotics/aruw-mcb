/*
 * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SWERVE_CHASSIS_SUBSYSTEM_MOCK_HPP_
#define SWERVE_CHASSIS_SUBSYSTEM_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/control/chassis/swerve_chassis_subsystem.hpp"

namespace aruwsrc
{
namespace mock
{
class SwerveChassisSubsystemMock : public aruwsrc::chassis::SwerveChassisSubsystem
{
public:
    SwerveChassisSubsystemMock(aruwsrc::Drivers *drivers);
    virtual ~SwerveChassisSubsystemMock();

    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, setDesiredOutput, (float, float, float), (override));
    MOCK_METHOD(void, setZeroRPM, ());
    MOCK_METHOD(float, chassisSpeedRotationPID, (float, float));
    MOCK_METHOD(void, refresh, (), (override));
    MOCK_METHOD(float, calculateRotationTranslationalGain, (float), ());
    MOCK_METHOD(int16_t, getLeftFrontRpmActual, (), (const override));
    MOCK_METHOD(int16_t, getLeftBackRpmActual, (), (const override));
    MOCK_METHOD(int16_t, getRightFrontRpmActual, (), (const override));
    MOCK_METHOD(int16_t, getRightBackRpmActual, (), (const override));
    MOCK_METHOD(float, getDesiredRotation, (), (const override));
};  // class SwerveChassisSubsystemMock
}  // namespace mock
}  // namespace aruwsrc

#endif  // SWERVE_CHASSIS_SUBSYSTEM_MOCK_HPP_
