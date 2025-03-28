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

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/sensors/current/analog_current_sensor.hpp"
#include "tap/drivers.hpp"

// #include "constants/chassis_constants.hpp"

#include "holonomic_chassis_subsystem.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include <gmock/gmock.h>

#include "tap/mock/dji_motor_mock.hpp"
#endif

namespace aruwsrc
{
namespace chassis
{
#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
using Motor = testing::NiceMock<tap::mock::DjiMotorMock>;
#else
using Motor = tap::motor::DjiMotor;
#endif

/**
 * Encapsulates a chassis with mecanum wheels in standard layout
 */
class Holonomic4MotorChassisSubsystem : public HolonomicChassisSubsystem
{
public:
    Holonomic4MotorChassisSubsystem(
        tap::Drivers* drivers,
        tap::communication::sensors::current::CurrentSensorInterface* currentSensor,
        Motor& leftFrontMotor,
        Motor& leftBackMotor,
        Motor& rightFrontMotor,
        Motor& rightBackMotor,
        tap::algorithms::SmoothPidConfig wheelVelocityPidConfig,
        can::capbank::CapacitorBank* capacitorBank = nullptr);

    inline bool allMotorsOnline() const override
    {
        return leftFrontMotor.isMotorOnline() && rightFrontMotor.isMotorOnline() &&
               leftBackMotor.isMotorOnline() && rightBackMotor.isMotorOnline();
    }

    virtual inline float getLeftFrontRpmActual() const
    {
        return leftFrontMotor.getEncoder()->getVelocity() / M_TWOPI * 60.f;
    }
    virtual inline float getLeftBackRpmActual() const
    {
        return leftBackMotor.getEncoder()->getVelocity() / M_TWOPI * 60.f;
    }
    virtual inline float getRightFrontRpmActual() const
    {
        return rightFrontMotor.getEncoder()->getVelocity() / M_TWOPI * 60.f;
    }
    virtual inline float getRightBackRpmActual() const
    {
        return rightBackMotor.getEncoder()->getVelocity() / M_TWOPI * 60.f;
    }

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

    void updateMotorRpmPid(int i);

    // wheel velocity PID variables
    tap::algorithms::SmoothPid velocityPid[4];

    float velocityPidErrors[4];

    // ✨ the motors ✨
    tap::motor::DjiMotor* motors[4];

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    Motor& leftFrontMotor;
    Motor& leftBackMotor;
    Motor& rightFrontMotor;
    Motor& rightBackMotor;

private:
#else
    Motor& leftFrontMotor;
    Motor& leftBackMotor;
    Motor& rightFrontMotor;
    Motor& rightBackMotor;
#endif
};

}  // namespace chassis
}  // namespace aruwsrc

#endif
