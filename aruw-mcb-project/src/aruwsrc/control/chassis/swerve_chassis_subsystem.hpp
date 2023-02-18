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
#include <array>

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
    /* SwerveChassisSubsystem(
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

    SwerveChassisSubsystem(
        aruwsrc::Drivers* drivers,
        unsigned int numModules,
        tap::motor::MotorId leftFrontAzimuthMotorId = LEFT_FRONT_AZIMUTH_MOTOR_ID,
        tap::motor::MotorId leftFrontDriveMotorId = LEFT_FRONT_MOTOR_ID,
        tap::motor::MotorId leftBackAzimuthMotorId = LEFT_BACK_AZIMUTH_MOTOR_ID,
        tap::motor::MotorId leftBackDriveMotorId = LEFT_BACK_MOTOR_ID,
        tap::motor::MotorId rightFrontAzimuthMotorId = RIGHT_FRONT_AZIMUTH_MOTOR_ID,
        tap::motor::MotorId rightFrontDriveMotorId = RIGHT_FRONT_MOTOR_ID,
        tap::motor::MotorId rightBackAzimuthMotorId = RIGHT_BACK_AZIMUTH_MOTOR_ID,
        tap::motor::MotorId rightBackDriveMotorId = RIGHT_BACK_MOTOR_ID,
        tap::gpio::Analog::Pin currentPin = CURRENT_SENSOR_PIN
    ); */

    SwerveChassisSubsystem(
        aruwsrc::Drivers* drivers,
        SwerveModuleConfig config1,
        SwerveModuleConfig config2,
        tap::gpio::Analog::Pin currentPin = CURRENT_SENSOR_PIN
    );

    SwerveChassisSubsystem(
        aruwsrc::Drivers* drivers,
        SwerveModuleConfig config1,
        SwerveModuleConfig config2,
        SwerveModuleConfig config3,
        SwerveModuleConfig config4,
        tap::gpio::Analog::Pin currentPin = CURRENT_SENSOR_PIN
    );



    void initialize() override;

    void setDesiredOutput(float x, float y, float r) override;

    void limitChassisPower() override;

    void refresh() override;

    inline int getNumChassisMotors() const override { return 8; }

    bool allMotorsOnline() const override;

    void setZeroRPM() override;

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
    
    // inline std::array<SwerveModule, MODULES>& createModules(aruwsrc::Drivers* drivers,
    //         std::array<SwerveModuleConfig, MODULES> moduleConfigs)
    // {
    //     std::array<SwerveModule, MODULES> moduleArray;
    //     for (unsigned int i = 0; i < MODULES; i++)
    //         moduleArray[i] = SwerveModule(drivers, moduleConfigs[i]);
    //     return moduleArray;
    // }

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    std::array<testing::NiceMock<aruwsrc::mock::SwerveModuleMock>, 4> modules;  
private:
#else
    std::array<chassis::SwerveModule, 4> modules;
#endif

    unsigned int NUM_MODULES;

    //extra debug stuff
    float lastXInput, lastYInput;

};  // class SwerveChassisSubsystem

}  // namespace chassis

}  // namespace aruwsrc

#endif  // SWERVE_CHASSIS_SUBSYSTEM_HPP_