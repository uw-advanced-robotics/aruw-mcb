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

/*
 * Copyright (c) 2019 Sanger_X
 */

#ifndef HOLONOMIC_4_MOTOR_CHASSIS_SUBSYSTEM_HPP_
#define HOLONOMIC_4_MOTOR_CHASSIS_SUBSYSTEM_HPP_

#include "tap/communication/gpio/analog.hpp"
#include "tap/communication/sensors/current/analog_current_sensor.hpp"
#include "tap/drivers.hpp"

#include "constants/chassis_constants.hpp"

#include "holonomic_chassis_subsystem.hpp"

namespace aruwsrc
{
namespace chassis
{
/**
 * Encapsulates a chassis with mecanum wheels in standard layout
 */
class Holonomic4MotorChassisSubsystem : public HolonomicChassisSubsystem
{
public:
    Holonomic4MotorChassisSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorId leftFrontMotorId = LEFT_FRONT_MOTOR_ID,
        tap::motor::MotorId leftBackMotorId = LEFT_BACK_MOTOR_ID,
        tap::motor::MotorId rightFrontMotorId = RIGHT_FRONT_MOTOR_ID,
        tap::motor::MotorId rightBackMotorId = RIGHT_BACK_MOTOR_ID,
        tap::gpio::Analog::Pin currentPin = CURRENT_SENSOR_PIN);

    inline bool allMotorsOnline() const override
    {
        return leftFrontMotor.isMotorOnline() && rightFrontMotor.isMotorOnline() &&
               leftBackMotor.isMotorOnline() && rightBackMotor.isMotorOnline();
    }

    inline int16_t getLeftFrontRpmActual() const override { return leftFrontMotor.getShaftRPM(); }
    inline int16_t getLeftBackRpmActual() const override { return leftBackMotor.getShaftRPM(); }
    inline int16_t getRightFrontRpmActual() const override { return rightFrontMotor.getShaftRPM(); }
    inline int16_t getRightBackRpmActual() const override { return rightBackMotor.getShaftRPM(); }

    inline int getNumChassisMotors() const override { return MODM_ARRAY_SIZE(motors); }

    void initialize() override;

    void setDesiredOutput(float x, float y, float r) override;

    void limitChassisPower() override;

    void refresh() override;

    modm::Matrix<float, 3, 1> getActualVelocityChassisRelative() const override;

private:
    /**
     * When you input desired x, y, an r rpm, this function translates
     * and sets the RPM of individual chassis motors.
     */
    void calculateOutput(float x, float y, float r, float maxWheelSpeed);

    void updateMotorRpmPid(
        modm::Pid<float>* pid,
        tap::motor::DjiMotor* const motor,
        float desiredRpm);

    // wheel velocity PID variables
    modm::Pid<float> velocityPid[4];

    // ✨ the motors ✨
    tap::motor::DjiMotor* motors[4];

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    testing::NiceMock<tap::mock::DjiMotorMock> leftFrontMotor;
    testing::NiceMock<tap::mock::DjiMotorMock> leftBackMotor;
    testing::NiceMock<tap::mock::DjiMotorMock> rightFrontMotor;
    testing::NiceMock<tap::mock::DjiMotorMock> rightBackMotor;

private:
#else
    // motors
    tap::motor::DjiMotor leftFrontMotor;
    tap::motor::DjiMotor leftBackMotor;
    tap::motor::DjiMotor rightFrontMotor;
    tap::motor::DjiMotor rightBackMotor;
#endif
};

}  // namespace chassis
}  // namespace aruwsrc

#endif