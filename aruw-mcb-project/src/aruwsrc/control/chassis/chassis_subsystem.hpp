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

#ifndef CHASSIS_SUBSYSTEM_HPP_
#define CHASSIS_SUBSYSTEM_HPP_

#include "tap/algorithms/extended_kalman.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/gpio/analog.hpp"
#include "tap/communication/sensors/current/analog_current_sensor.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

#include "tap/control/chassis/power_limiter.hpp"
#include "tap/motor/m3508_constants.hpp"
#include "tap/util_macros.hpp"

#include "aruwsrc/util_macros.hpp"
#include "modm/math/filter/pid.hpp"
#include "modm/math/matrix.hpp"

#include "chassis_constants.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc
{
namespace chassis
{
/**
 * This subsystem encapsulates a chassis with mecanum wheels (with a standard layout, e.g. not
 * swerve drive or similar).
 *
 * The chassis is in a right handed coordinate system with the x coordinate pointing torwards the
 * front of the chassis. As such, when looking down at the robot from above, the positive y
 * coordinate is to the left of the robot, and positive z is up. Also, the chassis rotation is
 * positive when rotating counterclockwise around the z axis.
 */
class ChassisSubsystem : public tap::control::chassis::ChassisSubsystemInterface
{
public:
    /**
     * Used to index into matrices returned by functions of the form get*Velocity*().
     */
    enum ChassisVelIndex
    {
        X = 0,
        Y = 1,
        R = 2,
    };

    static inline float getMaxWheelSpeed(bool refSerialOnline, int chassisPower)
    {
        if (!refSerialOnline)
        {
            chassisPower = 0;
        }

        // only re-interpolate when needed (since this function is called a lot and the chassis
        // power rarely changes, this helps cut down on unnecessary array searching/interpolation)
        if (lastComputedMaxWheelSpeed.first != chassisPower)
        {
            lastComputedMaxWheelSpeed.first = chassisPower;
            lastComputedMaxWheelSpeed.second =
                CHASSIS_POWER_TO_SPEED_INTERPOLATOR.interpolate(chassisPower);
        }

        return lastComputedMaxWheelSpeed.second;
    }

    ChassisSubsystem(
        aruwsrc::Drivers* drivers,
        tap::motor::MotorId leftFrontMotorId = LEFT_FRONT_MOTOR_ID,
        tap::motor::MotorId leftBackMotorId = LEFT_BACK_MOTOR_ID,
        tap::motor::MotorId rightFrontMotorId = RIGHT_FRONT_MOTOR_ID,
        tap::motor::MotorId rightBackMotorId = RIGHT_BACK_MOTOR_ID,
        tap::gpio::Analog::Pin currentPin = CURRENT_SENSOR_PIN);

    void initialize() override;

    /**
     * Updates the desired wheel RPM based on the passed in x, y, and r components of
     * movement. See the class comment for x and y terminology (should be in right hand coordinate
     * system).
     *
     * @param[in] x The desired velocity of the wheels to move in the x direction.
     *      So if x=1000, the chassis algorithm will attempt to apply 1000 RPM to motors
     *      in order to move the chassis forward.
     * @param[in] y The desired velocity of the wheels to move in the y direction.
     *      See x param for further description.
     * @param[in] r The desired velocity of the wheels to rotate the chassis.
     *      See x param for further description.
     */
    mockable void setDesiredOutput(float x, float y, float r);

    /**
     * Zeros out the desired motor RPMs for all motors, but importantly doesn't zero out any other
     * chassis state information like desired rotation.
     */
    mockable void setZeroRPM();

    /**
     * Run chassis rotation PID on some actual turret angle offset.
     *
     * @param currentAngleError The error as an angle. For autorotation,
     * error between gimbal and center of chassis.
     * @param errD The derivative of currentAngleError.
     *
     * @retval a desired rotation speed (wheel speed)
     */
    mockable float chassisSpeedRotationPID(float currentAngleError, float errD);

    void refresh() override;

    /**
     * When the desired rotational wheel speed is large, you can slow down your translational speed
     * to make a tighter and more controllable turn. This function that can be used to scale down
     * the translational chassis speed based on the desired rotational wheel speed.
     *
     * @param chassisRotationDesiredWheelspeed The desired rotational component of the chassis, in
     * wheel RPM.
     * @return A value between [0, 1] that is inversely proportional to the square of
     * chassisRotationDesiredWheelspeed. You then multiply your desired translational RPM by this
     * value.
     */
    mockable float calculateRotationTranslationalGain(float chassisRotationDesiredWheelspeed);

    const char* getName() override { return "Chassis"; }

    /**
     * @return The desired chassis velocity in chassis relative frame, as a vector <vx, vy, vz>,
     *      where vz is rotational velocity. This is the desired velocity calculated before any
     *      sort of limiting occurs (other than base max RPM limiting). Units: m/s
     * @note Equations slightly modified from this paper:
     *      https://www.hindawi.com/journals/js/2015/347379/.
     */
    modm::Matrix<float, 3, 1> getDesiredVelocityChassisRelative() const;

    /**
     * @return The actual chassis velocity in chassis relative frame, as a vector <vx, vy, vz>,
     *      where vz is rotational velocity. This is the velocity calculated from the chassis's
     *      encoders. Units: m/s
     */
    modm::Matrix<float, 3, 1> getActualVelocityChassisRelative() const;

    /**
     * Transforms the chassis relative velocity of the form <vx, vy, vz> into world relative frame,
     * given some particular chassis heading (z direction, assumed to be in radians). Transforms
     * the input matrix chassisRelativeVelocity. Units: m/s
     */
    void getVelocityWorldRelative(
        modm::Matrix<float, 3, 1>& chassisRelativeVelocity,
        float chassisHeading) const;

    inline int getNumChassisMotors() const override { return MODM_ARRAY_SIZE(motors); }

    inline int16_t getLeftFrontRpmActual() const override { return leftFrontMotor.getShaftRPM(); }
    inline int16_t getLeftBackRpmActual() const override { return leftBackMotor.getShaftRPM(); }
    inline int16_t getRightFrontRpmActual() const override { return rightFrontMotor.getShaftRPM(); }
    inline int16_t getRightBackRpmActual() const override { return rightBackMotor.getShaftRPM(); }

    inline bool allMotorsOnline() const override
    {
        return leftFrontMotor.isMotorOnline() && rightFrontMotor.isMotorOnline() &&
               leftBackMotor.isMotorOnline() && rightBackMotor.isMotorOnline();
    }

    void onHardwareTestStart() override;

    mockable float getDesiredRotation() const { return desiredRotation; }

private:
    static modm::Pair<int, float> lastComputedMaxWheelSpeed;

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

    // wheel velocity PID variables
    modm::Pid<float> velocityPid[4];

    /**
     * Stores the desired RPM of each of the motors in a matrix, indexed by WheelRPMIndex
     */
    modm::Matrix<float, 4, 1> desiredWheelRPM;

    modm::Matrix<float, 3, 4> wheelVelToChassisVelMat;

    float desiredRotation = 0;

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

    tap::motor::DjiMotor* motors[4];

    tap::communication::sensors::current::AnalogCurrentSensor currentSensor;

    tap::control::chassis::PowerLimiter chassisPowerLimiter;

    /**
     * When you input desired x, y, an r values, this function translates
     * and sets the RPM of individual chassis motors.
     */
    void mecanumDriveCalculate(float x, float y, float r, float maxWheelSpeed);

    void updateMotorRpmPid(
        modm::Pid<float>* pid,
        tap::motor::DjiMotor* const motor,
        float desiredRpm);

    void limitChassisPower();

    /**
     * Converts the velocity matrix from raw RPM to wheel velocity in m/s.
     */
    inline modm::Matrix<float, 4, 1> convertRawRPM(const modm::Matrix<float, 4, 1>& mat) const
    {
        static constexpr float ratio = 2.0f * M_PI * CHASSIS_GEARBOX_RATIO / 60.0f;
        return mat * ratio;
    }
};  // class ChassisSubsystem

}  // namespace chassis

}  // namespace aruwsrc

#endif  // CHASSIS_SUBSYSTEM_HPP_
