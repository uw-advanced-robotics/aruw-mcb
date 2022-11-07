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

#ifndef SWERVE_MODULE_HPP_
#define SWERVE_MODULE_HPP_

#include "tap/motor/m3508_constants.hpp"

#include "modm/math/filter/pid.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

using Motor = tap::motor::DjiMotor;

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc
{
namespace chassis
{
/**
 *
 * This class encapsultes a swerve module using two motors.
 * Input is in meters per second and radians.
 *
 */
class SwerveModule
{
    struct SwerveModuleConfig
    {
        // Whether any motor is inverted
        bool driveMotorInverted, azimuthMotorInverted;
        // Gear ratios for motors
        float driveMotorGearing, azimuthMotorGearing;

        


    };

public:
    SwerveModule(
        tap::motor::MotorId driveMotorId,
        tap::motor::MotorId azimuthMotorId,
        const SwerveModuleConfig& swerveModuleConfig);

    void setDesiredState(float metersPerSecond, float radianOutput);

    float getVelocity();

    float getAngle();

private:
    modm::Pid<float> drivePid;
    modm::Pid<float> azimuthPid;

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    testing::NiceMock<tap::mock::DjiMotorMock> driveMotor;
    testing::NiceMock<tap::mock::DjiMotorMock> azimuthMotor;

private:
#else
    // motors
    Motor driveMotor;
    Motor azimuthMotor;

#endif
};  // class SwerveModule

}  // namespace chassis

}  // namespace aruwsrc

#endif  // SWERVE_MODULE_HPP_
