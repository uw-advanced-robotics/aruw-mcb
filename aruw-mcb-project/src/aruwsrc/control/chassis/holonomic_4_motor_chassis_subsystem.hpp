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
        tap::communication::sensors::current::CurrentSensorInterface* currentSensor,
        can::capbank::CapacitorBank* capacitorBank = nullptr,
        tap::motor::MotorId leftFrontMotorId = LEFT_FRONT_MOTOR_ID,
        tap::motor::MotorId leftBackMotorId = LEFT_BACK_MOTOR_ID,
        tap::motor::MotorId rightFrontMotorId = RIGHT_FRONT_MOTOR_ID,
        tap::motor::MotorId rightBackMotorId = RIGHT_BACK_MOTOR_ID);

    inline bool allMotorsOnline() const override
    {
        return leftFrontMotor.isMotorOnline() && rightFrontMotor.isMotorOnline() &&
               leftBackMotor.isMotorOnline() && rightBackMotor.isMotorOnline();
    }

    virtual inline int16_t getLeftFrontRpmActual() const { return leftFrontMotor.getShaftRPM(); }
    virtual inline int16_t getLeftBackRpmActual() const { return leftBackMotor.getShaftRPM(); }
    virtual inline int16_t getRightFrontRpmActual() const { return rightFrontMotor.getShaftRPM(); }
    virtual inline int16_t getRightBackRpmActual() const { return rightBackMotor.getShaftRPM(); }

    inline int getNumChassisMotors() const override { return MODM_ARRAY_SIZE(motors); }

    void initialize() override;

    void setDesiredOutput(float x, float y, float r) override;

    void limitChassisPower() override;

    inline void setZeroRPM() override { desiredWheelRPM = desiredWheelRPM.zeroMatrix(); }

    /**
     * Used to index into the desiredWheelRPM matrix and velocityPid array.
     */
    enum WheelRPMIndex
    {
        LF = 0,
        RF = 1,
        LB = 2,
        RB = 3,
    };

    void refresh() override;

    void refreshSafeDisconnect() override
    {
        for (int i = 0; i < getNumChassisMotors(); i++)
        {
            motors[i]->setDesiredOutput(0);
        }
    }

    modm::Matrix<float, 3, 1> getActualVelocityChassisRelative() const override;

    /**
     * Stores the desired RPM of each of the motors in a matrix, indexed by WheelRPMIndex
     */
    modm::Matrix<float, 4, 1> desiredWheelRPM;

    /**
     * @return The desired chassis velocity in chassis relative frame, as a vector <vx, vy, vz>,
     *      where vz is rotational velocity. This is the desired velocity calculated before any
     *      sort of limiting occurs (other than base max RPM limiting). Units: m/s
     * @note Equations slightly modified from this paper:
     *      https://www.hindawi.com/journals/js/2015/347379/.
     */
    mockable modm::Matrix<float, 3, 1> getDesiredVelocityChassisRelative() const;

    float mpsToRpm(float mps) const override
    {
        return mps / (M_TWOPI * WHEEL_RADIUS) * 60.0f / CHASSIS_GEARBOX_RATIO;
    }

protected:
    modm::Matrix<float, 3, 4> wheelVelToChassisVelMat;

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

    float powerLimitFrac;

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
