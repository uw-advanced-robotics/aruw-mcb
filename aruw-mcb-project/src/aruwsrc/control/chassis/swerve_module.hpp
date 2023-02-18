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

#ifndef SWERVE_MODULE_HPP_
#define SWERVE_MODULE_HPP_

#include "tap/motor/m3508_constants.hpp"

#include "constants/chassis_constants.hpp"
#include "modm/math/filter/pid.hpp"
#include "tap/algorithms/smooth_pid.hpp"
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

/**
 *
 * This class encapsultes a swerve module using two motors.
 * Input is in meters per second and radians.
 *
 */
class SwerveModule
{
public:
    // SwerveModule(
    //     aruwsrc::Drivers* drivers,
    //     tap::motor::MotorId driveMotorId,
    //     tap::motor::MotorId azimuthMotorId,
    //     float positionWithinChassisX,
    //     float positionWithinChassisY,
    //     SwerveModuleConfig& swerveModuleConfig = SWERVE_CONFIG,
    //     tap::algorithms::SmoothPidConfig azimuthPidConfig = SWERVE_CONFIG.azimuthPidConfig);
    
    SwerveModule(
        aruwsrc::Drivers* drivers,
        SwerveModuleConfig& swerveModuleConfig);

    const float ANGULAR_ERROR_POWER_BIAS = M_PI_2 / 4.5f;

    void setDesiredState(float metersPerSecond, float radianOutput);

    void scaleAndSetDesiredState(float scaleCoeff);

    float calculate(float x, float y, float r);

    float getDriveVelocity() const;

    float getDriveRPM() const;

    float getAngle() const;

    float getAngularVelocity() const;

    void initialize();

    void calibrateAzimuth();

    void refresh();

    float calculateTotalModuleError() const;

    void setZeroRPM();

    float getAzimuthError() const;
    float getDriveError() const;
    bool allMotorsOnline() const;

    std::vector<float> getModuleVelocity();

    void limitPower(float frac);
    
    float mpsToRpm(float mps) const;
    float rpmToMps(float rpm) const;

private:

    float optimizeAngle(float desiredAngle);

    inline float unwrapAngle(float angle, float denomination)
    {
        return fmod(fmod(angle, denomination) + M_TWOPI, denomination);
        //double % needed to ensure output is positive, bc % can be negative
    }

    
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
#endif

    //int64_t azimuthZeroOffset = 0;

    bool isCalibrated = false;


    modm::Pid<float> drivePid;
    tap::algorithms::SmoothPid azimuthPid;

    float preScaledSpeedSetpoint{0}, rotationSetpointRadians{0};
    float speedSetpointRPM, rotationSetpoint;
    
    //handles wrapping desired rotation and reversing module (in radians, will always be a multiple of PI)
    float rotationOffset{0}, wrappingRotationOffset{0}, reversingRotationOffset{0};
    bool reversed{false};

    const SwerveModuleConfig config;

    //extra debug stuff
    float drivePIDOutput, driveOutputDesired;
    int desiredAziWrapNum = 0;
};  // class SwerveModule

}  // namespace chassis

}  // namespace aruwsrc

#endif  // SWERVE_MODULE_HPP_
