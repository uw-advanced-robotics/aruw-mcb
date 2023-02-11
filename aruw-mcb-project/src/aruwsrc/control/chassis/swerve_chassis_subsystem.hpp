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

#ifndef SWERVE_CHASSIS_SUBSYSTEM_HPP_
#define SWERVE_CHASSIS_SUBSYSTEM_HPP_


#include "tap/algorithms/extended_kalman.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/gpio/analog.hpp"
#include "tap/communication/sensors/current/analog_current_sensor.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/control/chassis/power_limiter.hpp"
#include "tap/motor/m3508_constants.hpp"
#include "tap/util_macros.hpp"

#include "aruwsrc/util_macros.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/swerve_module.hpp"
#include "constants/chassis_constants.hpp"
#include "modm/math/filter/pid.hpp"
#include "modm/math/matrix.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "aruwsrc/mock/swerve_module_mock.hpp"
#endif


namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc
{
namespace chassis
{

class SwerveChassisSubsystem : public chassis::HolonomicChassisSubsystem
{
public:
    SwerveChassisSubsystem(
        aruwsrc::Drivers* drivers,
        tap::motor::MotorId leftFrontAzimuthMotorId = LEFT_FRONT_AZIMUTH_MOTOR_ID,
        tap::motor::MotorId leftFrontDriveMotorId = LEFT_FRONT_MOTOR_ID,
        tap::motor::MotorId leftBackAzimuthMotorId = LEFT_BACK_AZIMUTH_MOTOR_ID,
        tap::motor::MotorId leftBackDriveMotorId = LEFT_BACK_MOTOR_ID,
        tap::motor::MotorId rightFrontAzimuthMotorId = RIGHT_FRONT_AZIMUTH_MOTOR_ID,
        tap::motor::MotorId rightFrontDriveMotorId = RIGHT_FRONT_MOTOR_ID,
        tap::motor::MotorId rightBackAzimuthMotorId = RIGHT_BACK_AZIMUTH_MOTOR_ID,
        tap::motor::MotorId rightBackDriveMotorId = RIGHT_BACK_MOTOR_ID,
        tap::gpio::Analog::Pin currentPin = CURRENT_SENSOR_PIN
    );

    void initialize() override;

    void setDesiredOutput(float x, float y, float r) override;

    void limitChassisPower() override;

    void refresh() override;

    inline int getNumChassisMotors() const override { return 8; }

    inline bool allMotorsOnline() const override
    {
        return modules[0].allMotorsOnline() &&
            modules[1].allMotorsOnline() &&
            modules[2].allMotorsOnline() &&
            modules[3].allMotorsOnline();
    }

    inline void setZeroRPM() override
    {
        modules[0].setZeroRPM();
        modules[1].setZeroRPM();
        modules[2].setZeroRPM();
        modules[3].setZeroRPM();
    }

    /**
     * Used to index into the desiredWheelRPM matrix and velocityPid array.
     */
    enum ModuleIndex
    {
        LF = 0,
        LB = 1,
        RF = 2,
        RB = 3,
    };

    /**
     * Stores the desired state of each of the modules in a matrix, indexed by ModuleIndex
     */
    modm::Matrix<float, 4, 1> desiredModuleSpeeds;

    modm::Matrix<float, 3, 8> swerveWheelVelToChassisVelMat;

    modm::Matrix<float, 3, 1> getActualVelocityChassisRelative() const override;

    modm::Matrix<float, 3, 1> getDesiredVelocityChassisRelative() const;

    //only to satisfy chassis subsystem interface
    inline int16_t getLeftFrontRpmActual() const override { return modules[LF].getDriveRPM(); }
    inline int16_t getLeftBackRpmActual() const override { return modules[LB].getDriveRPM(); }
    inline int16_t getRightFrontRpmActual() const override { return modules[RF].getDriveRPM(); }
    inline int16_t getRightBackRpmActual() const override { return modules[RB].getDriveRPM(); }

private:

    /**
     * When you input desired x, y, an r rpm, this function translates
     * and sets the target azimuth and drive RPM of individual chassis modules.
     */
    void swerveDriveCalculate(float x, float y, float r, float maxWheelSpeed);

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    testing::NiceMock<aruwsrc::mock::SwerveModuleMock> modules[4];  
private:
#else
    chassis::SwerveModule* modules[4];  
#endif      

};  // class SwerveChassisSubsystem

}  // namespace chassis

}  // namespace aruwsrc

#endif  // SWERVE_CHASSIS_SUBSYSTEM_HPP_