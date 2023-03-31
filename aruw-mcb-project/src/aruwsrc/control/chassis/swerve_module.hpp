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

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/m3508_constants.hpp"

#include "swerve_module_config.hpp"
#include "aruwsrc/algorithms/wheel.hpp"
#include "constants/chassis_constants.hpp"
#include "modm/math/filter/pid.hpp"
#include "modm/math/geometry/angle.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

using Motor = tap::motor::DjiMotor;
using Wheel = aruwsrc::algorithms::Wheel;

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
 * This class encapsulates a swerve module with two motors.
 *
 */
class SwerveModule
{
public:
    SwerveModule(
        tap::Drivers* drivers,
        SwerveModuleConfig& swerveModuleConfig = DEFAULT_SWERVE_CONFIG);

    /**
     * uses the internally stored values from calculate() to update 
     * the desired module state, scaling the speed
     * @param scaleCoeff the scale factor applied to the module speed
    */
    void scaleAndSetDesiredState(float scaleCoeff);

    /**
     * computes initial candidate for module state
     * @param x desired chassis x velocity in m/s
     * @param y desired chassis y velocity in m/s
     * @param r desired chassis angular velocity in rad/s
     * @return pre-scaled module speed in rpm
     */
    float calculate(float x, float y, float r);

    /**
     * Returns MPS of the wheel
     */
    float getDriveVelocity() const;

    /**
     * Returns RPM of the wheel
     */
    float getDriveRPM() const;

    /**
     * This returns Radian position of azimuth motor, CCW+
     */
    float getAngle() const;

    /**
     * This returns deg/sec velocity of azimuth motor, CCW+
     */
    float getAngularVelocity() const;

    void initialize();

    /**
     * Updates drive and azimuth PIDs
     */
    void refresh();

    void setZeroRPM();

    bool allMotorsOnline() const;

    inline modm::Matrix<float, 2, 1> getModuleVelocity() const
    {
        float mag = getDriveVelocity();
        float ang = getAngle();
        modm::Matrix<float, 2, 1> velocity;
        velocity[0][0] = mag * cos(ang);
        velocity[1][0] = mag * sin(ang);
        return velocity;
    }

    // limits power to both motors equally
    void limitPower(float frac);

    // in radians
    inline float getRotationSetpoint() { return rotationSetpoint; }
    // in rpm
    inline float getSpeedSetpoint() { return speedSetpointRPM; }

    const Wheel wheel;

// motors
#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    testing::NiceMock<tap::mock::DjiMotorMock> driveMotor;
    testing::NiceMock<tap::mock::DjiMotorMock> azimuthMotor;

private:
#else
private:
    Motor driveMotor;
    Motor azimuthMotor;
#endif

    /**
     * sets the module's desired rotation and wheel speed
     * @param metersPerSecond desired module speed in mps
     * @param radianOutput desired module rotation in radians
    */
    void setDesiredState(float metersPerSecond, float radianOutput);

    inline float wrapAngle(float angle, float denomination)
    {
        return fmod(
            fmod(angle, denomination) + M_TWOPI,
            denomination);  // replace M_TWOPI with denomination? doesn't matter for its one use
                            // case currently
        // double fmod needed to ensure output is positive bc fmod can be negative
    }

    const SwerveModuleConfig config;

    modm::Pid<float> drivePid;
    tap::algorithms::SmoothPid azimuthPid;

    const float rotationVectorX, rotationVectorY;
    float preScaledSpeedSetpoint{0}, preScaledRotationSetpoint{0}, speedSetpointRPM,
        rotationSetpoint;

    // handles wrapping desired rotation and reversing module (in radians, will always be a multiple
    // of PI)
    float rotationOffset{0};

    modm::interpolation::Linear<modm::Pair<float, float>> angularBiasLUTInterpolator;

};  // class SwerveModule

}  // namespace chassis

}  // namespace aruwsrc

#endif  // SWERVE_MODULE_HPP_
