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

#include "constants/chassis_constants.hpp"
#include "modm/math/filter/pid.hpp"
#include "modm/math/geometry/angle.hpp"

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

struct SwerveModuleConfig
{
    const float WHEEL_DIAMETER_M = 0.076f;
    const float WHEEL_CIRCUMFRENCE_M = WHEEL_DIAMETER_M * M_PI;

    // Whether any motor is inverted
    const bool driveMotorInverted, azimuthMotorInverted;
    // Gear ratios for motors
    const float driveMotorGearing, azimuthMotorGearing;

    const float drivePidKp = 0.0f;
    const float drivePidKi = 0.0f;
    const float drivePidKd = 0.0f;
    const float drivePidMaxIntegralErrorSum = 0.0f;
    const float drivePidMaxOutput = 16'384.0f;
    const float drivePidFeedForwardConstant = 0.0f;

    const float azimuthPidKp = 0.0f;
    const float azimuthPidKi = 0.0f;
    const float azimuthPidKd = 0.0f;
    const float azimuthPidMaxIntegralErrorSum = 0.0f;
    const float azimuthPidMaxOutput = 16'384.0f;
    const float azimuthPidFeedForwardConstant = 0.0f;
};

/**
 *
 * This class encapsultes a swerve module using two motors.
 * Input is in meters per second and radians.
 *
 */
class SwerveModule
{
public:
    SwerveModule(
        aruwsrc::Drivers* drivers,
        tap::motor::MotorId driveMotorId,
        tap::motor::MotorId azimuthMotorId,
        SwerveModuleConfig& swerveModuleConfig,
        float positionWithinChassisX,
        float positionWithinChassisY);

    const float ANGULAR_ERROR_POWER_BIAS = M_PI_2 / 4.5f;

    void setDesiredState(float metersPerSecond, float radianOutput);

    void scaleAndSetDesiredState(float scaleCoeff);

    float calculate(float x, float y, float r);

    float getDriveVelocity() const;

    float getAngle() const;

    void initialize();

    void zeroAzimuth();

    void refresh();

    float calculateTotalModuleError() const;

    

    float getAzimuthError() const;
    float getDriveError() const;
    bool allMotorsOnline() const;

    std::vector<float> getModuleVelocity();

    void limitPower(float frac);
    

private:
    float mpsToRpm(float mps) const;
    float rpmToMps(float rpm) const;

    float optimizeAngle(float desiredAngle);

    
    float rotationVectorX;
    float rotationVectorY;

    

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    testing::NiceMock<tap::mock::DjiMotorMock> driveMotor;
    testing::NiceMock<tap::mock::DjiMotorMock> azimuthMotor;

private:
#else
    // motors
    Motor driveMotor;
    Motor azimuthMotor;
    int64_t azimuthZeroOffset = 0;

    modm::Pid<float> drivePid;
    modm::Pid<float> azimuthPid;

    float preScaledSpeedSetpoint{0}, preOptimizedRotationSetpoint{0};
    float speedSetpoint, rotationSetpoint;
    SwerveModuleConfig config;

#endif
};  // class SwerveModule

}  // namespace chassis

}  // namespace aruwsrc

#endif  // SWERVE_MODULE_HPP_
