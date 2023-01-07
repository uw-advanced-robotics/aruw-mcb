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
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
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
        tap::motor::MotorId leftFrontAzimuthMotorId,
        tap::motor::MotorId leftFrontDriveMotorId,
        tap::motor::MotorId leftBackAzimuthMotorId,
        tap::motor::MotorId leftBackDriveMotorId,
        tap::motor::MotorId rightFrontAzimuthMotorId,
        tap::motor::MotorId rightFrontDriveMotorId,
        tap::motor::MotorId rightBackAzimuthMotorId,
        tap::motor::MotorId rightBackDriveMotorId,
        chassis::SwerveModuleConfig config,
        tap::gpio::Analog::Pin currentPin
    );

    void initialize() override;

    void setDesiredOutput(float x, float y, float r) override;

    void limitChassisPower() override;

    void refresh() override;

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
    modm::Matrix<float, 4, 2> desiredModuleStates;

    modm::Matrix<float, 3, 8> swerveWheelVelToChassisVelMat;

    modm::Matrix<float, 3, 1> getActualVelocityChassisRelative() const override;

private:

    /**
     * When you input desired x, y, an r rpm, this function translates
     * and sets the target azimuth and drive RPM of individual chassis modules.
     */
    void swerveDriveCalculate(float x, float y, float r, float maxWheelSpeed);

    chassis::SwerveModule modules[4];        

};  // class SwerveChassisSubsystem

}  // namespace chassis

}  // namespace aruwsrc

#endif  // SWERVE_CHASSIS_SUBSYSTEM_HPP_