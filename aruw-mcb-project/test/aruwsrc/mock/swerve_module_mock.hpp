/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SWERVE_MODULE_MOCK_HPP_
#define SWERVE_MODULE_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/control/chassis/swerve_module.hpp"
#include "aruwsrc/control/chassis/swerve_module_config.hpp"

namespace aruwsrc
{
namespace mock
{
class SwerveModuleMock : public aruwsrc::chassis::SwerveModule
{
public:
    SwerveModuleMock(tap::Drivers *drivers);

    SwerveModuleMock(tap::Drivers *drivers, aruwsrc::chassis::SwerveModuleConfig &config);
    virtual ~SwerveModuleMock();

    MOCK_METHOD(void, setDesiredState, (float, float), ());
    MOCK_METHOD(void, scaleAndSetDesiredState, (float), ());
    MOCK_METHOD(float, calculate, (float, float, float));
    MOCK_METHOD(float, getDriveVelocity, (float, float), (const));
    MOCK_METHOD(float, getAngle, (), (const));
    MOCK_METHOD(void, initialize, (), ());
    MOCK_METHOD(void, refresh, (), ());
    MOCK_METHOD(float, calculateTotalModuleError, (), (const));
    MOCK_METHOD(float, getAzimuthError, (), (const));
    MOCK_METHOD(float, getDriveError, (), (const));
    MOCK_METHOD(bool, allMotorsOnline, (), (const));
    // getModuleVelocity
    MOCK_METHOD(void, limitPower, (float), ());

};  // class SwerveModuleMock
}  // namespace mock
}  // namespace aruwsrc

#endif  // SWERVE_MODULE_MOCK_HPP_
