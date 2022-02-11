/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc
{
namespace chassis
{
/**
 * This subsystem encapsulates a chassis with mecanum wheels (with a standard
 * layout, e.g. not swerve drive or similar).
 *
 * Terminology:
 *     - Anywhere 'x' is used, we mean this to be when looking down
 *       at the robot, the vertical axis, and likewise, 'y' is the
 *       horizontal axis of the robot.
 *     - In other words, 'x' is the bow/stern and 'y' is starboard/
 *       port in boat terms.
 */
class ChassisSubsystem : public tap::control::chassis::ChassisSubsystemInterface
{
public:
    /**
     * Max wheel speed, measured in RPM of the encoder (rather than shaft)
     * we use this for wheel speed since this is how dji's motors measures motor speed.
     */
    static constexpr int MIN_WHEEL_SPEED_SINGLE_MOTOR = 4000;
    static constexpr int MAX_WHEEL_SPEED_SINGLE_MOTOR = 8000;
    static constexpr int MIN_CHASSIS_POWER = 40;
    static constexpr int MAX_CHASSIS_POWER = 120;
    static constexpr int WHEEL_SPEED_OVER_CHASSIS_POWER_SLOPE =
        (MAX_WHEEL_SPEED_SINGLE_MOTOR - MIN_WHEEL_SPEED_SINGLE_MOTOR) /
        (MAX_CHASSIS_POWER - MIN_CHASSIS_POWER);
    static_assert(WHEEL_SPEED_OVER_CHASSIS_POWER_SLOPE >= 0);

    static inline float getMaxUserWheelSpeed(bool refSerialOnline, int chassisPower)
    {
        if (refSerialOnline)
        {
            float desWheelSpeed = WHEEL_SPEED_OVER_CHASSIS_POWER_SLOPE *
                                      static_cast<float>(chassisPower - MIN_CHASSIS_POWER) +
                                  MIN_WHEEL_SPEED_SINGLE_MOTOR;

            return tap::algorithms::limitVal(
                desWheelSpeed,
                static_cast<float>(MIN_WHEEL_SPEED_SINGLE_MOTOR),
                static_cast<float>(MAX_WHEEL_SPEED_SINGLE_MOTOR));
        }
        else
        {
            return MIN_WHEEL_SPEED_SINGLE_MOTOR;
        }
    }

    /**
     * The minimum desired wheel speed for chassis rotation, measured in RPM before
     * we start slowing down translational speed.
     */
    static constexpr float MIN_ROTATION_THRESHOLD = 800.0f;

    /**
     * Pin to use for current sensing
     */
    static constexpr tap::gpio::Analog::Pin CURRENT_SENSOR_PIN = tap::gpio::Analog::Pin::S;

    /// @see power_limiter.hpp for what these mean
    static constexpr float STARTING_ENERGY_BUFFER = 60.0f;
    static constexpr float ENERGY_BUFFER_LIMIT_THRESHOLD = 60.0f;
    static constexpr float ENERGY_BUFFER_CRIT_THRESHOLD = 10.0f;
    static constexpr uint16_t POWER_CONSUMPTION_THRESHOLD = 20;
    static constexpr float CURRENT_ALLOCATED_FOR_ENERGY_BUFFER_LIMITING = 30000;

private:
#if defined(ALL_SOLDIERS)
    /**
     * Velocity PID gains and constants.
     */
    static constexpr float VELOCITY_PID_KP = 22.0f;
    static constexpr float VELOCITY_PID_KI = 0.2f;
    static constexpr float VELOCITY_PID_KD = 0.0f;
    static constexpr float VELOCITY_PID_MAX_ERROR_SUM = 5'000.0f;
    /**
     * This max output is measured in the c620 robomaster translated current.
     * Per the datasheet, the controllable current range is -16384 ~ 0 ~ 16384.
     * The corresponding speed controller output torque current range is
     * -20 ~ 0 ~ 20 A.
     */
    static constexpr float VELOCITY_PID_MAX_OUTPUT = 16000.0f;

    /**
     * Rotation PID: A PD controller for chassis autorotation. The PID parameters for the
     * controller are listed below.
     */
    static constexpr float AUTOROTATION_PID_KP = 180.0f;
    static constexpr float AUTOROTATION_PID_KD = 4000.0f;
    static constexpr float AUTOROTATION_PID_MAX_P = 5000.0f;
    static constexpr float AUTOROTATION_PID_MAX_D = 5000.0f;
    static constexpr float AUTOROTATION_PID_MAX_OUTPUT = 5500.0f;

    // mechanical chassis constants, all in m
    /**
     * Radius of the wheels (m).
     */
    static constexpr float WHEEL_RADIUS = 0.076;
    /**
     * Distance from center of the two front wheels (m).
     */
    static constexpr float WIDTH_BETWEEN_WHEELS_Y = 0.366f;
    /**
     * Distance from center of the front and rear wheels (m).
     */
    static constexpr float WIDTH_BETWEEN_WHEELS_X = 0.366f;
    /**
     * Gimbal offset from the center of the chassis, see note above for explanation of x and y (m).
     */
    static constexpr float GIMBAL_X_OFFSET = 0.0f;
    /**
     * @see `GIMBAL_X_OFFSET`.
     */
    static constexpr float GIMBAL_Y_OFFSET = 0.0f;
    static constexpr float CHASSIS_GEARBOX_RATIO = (1.0f / 19.0f);

#elif defined(TARGET_HERO)
    /**
     * Velocity PID gains and constants.
     */
    static constexpr float VELOCITY_PID_KP = 20.0f;
    static constexpr float VELOCITY_PID_KI = 0.0f;
    static constexpr float VELOCITY_PID_KD = 0.0f;
    static constexpr float VELOCITY_PID_MAX_ERROR_SUM = 0.0f;
    static constexpr float VELOCITY_PID_MAX_OUTPUT = 16000.0f;

    /**
     * Rotation PID: A PD controller for chassis autorotation. The PID parameters for the
     * controller are listed below.
     */
    static constexpr float AUTOROTATION_PID_KP = 210.0f;
    static constexpr float AUTOROTATION_PID_KD = 4500.0f;
    static constexpr float AUTOROTATION_PID_MAX_P = 5000.0f;
    static constexpr float AUTOROTATION_PID_MAX_D = 5000.0f;
    static constexpr float AUTOROTATION_PID_MAX_OUTPUT = 5500.0f;

    // mechanical chassis constants
    /**
     * Radius of the wheels.
     */
    static constexpr float WHEEL_RADIUS = 0.076f;
    /**
     * Distance from center of the two front wheels.
     */
    static constexpr float WIDTH_BETWEEN_WHEELS_Y = 0.46f;
    /**
     * Distance from center of the front and rear wheels.
     */
    static constexpr float WIDTH_BETWEEN_WHEELS_X = 0.46f;
    /**
     * Gimbal offset from the center of the chassis, see note above for explanation of x and y.
     */
    static constexpr float GIMBAL_X_OFFSET = 0.0f;
    /**
     * @see `GIMBAL_X_OFFSET`.
     */
    static constexpr float GIMBAL_Y_OFFSET = 0.0f;
    static constexpr float CHASSIS_GEARBOX_RATIO = (1.0f / 19.0f);

#else
    /**
     * Velocity PID gains and constants.
     */
    static constexpr float VELOCITY_PID_KP = 0.0f;
    static constexpr float VELOCITY_PID_KI = 0.0f;
    static constexpr float VELOCITY_PID_KD = 0.0f;
    static constexpr float VELOCITY_PID_MAX_ERROR_SUM = 0.0f;
    static constexpr float VELOCITY_PID_MAX_OUTPUT = 0.0f;

    /**
     * Rotation PID: A PD controller for chassis autorotation. The PID parameters for the
     * controller are listed below.
     */
    static constexpr float AUTOROTATION_PID_KP = 210.0f;
    static constexpr float AUTOROTATION_PID_KD = 4500.0f;
    static constexpr float AUTOROTATION_PID_MAX_P = 5000.0f;
    static constexpr float AUTOROTATION_PID_MAX_D = 5000.0f;
    static constexpr float AUTOROTATION_PID_MAX_OUTPUT = 5500.0f;

    // mechanical chassis constants
    /**
     * Radius of the wheels.
     */
    static constexpr float WHEEL_RADIUS = 0.0f;
    /**
     * Distance from center of the two front wheels.
     */
    static constexpr float WIDTH_BETWEEN_WHEELS_Y = 0.0f;
    /**
     * Distance from center of the front and rear wheels.
     */
    static constexpr float WIDTH_BETWEEN_WHEELS_X = 0.0f;
    /**
     * Gimbal offset from the center of the chassis, see note above for explanation of x and y.
     */
    static constexpr float GIMBAL_X_OFFSET = 0.0f;
    /**
     * @see `GIMBAL_X_OFFSET`
     */
    static constexpr float GIMBAL_Y_OFFSET = 0.0f;
    static constexpr float CHASSIS_GEARBOX_RATIO = (1.0f / 19.0f);

#endif

public:
    // hardware constants, not specific to any particular chassis
    static constexpr tap::motor::MotorId LEFT_FRONT_MOTOR_ID = tap::motor::MOTOR2;
    static constexpr tap::motor::MotorId LEFT_BACK_MOTOR_ID = tap::motor::MOTOR3;
    static constexpr tap::motor::MotorId RIGHT_FRONT_MOTOR_ID = tap::motor::MOTOR1;
    static constexpr tap::motor::MotorId RIGHT_BACK_MOTOR_ID = tap::motor::MOTOR4;

    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS2;

    // wheel velocity PID variables
    modm::Pid<float> velocityPid[4];

    /**
     * Used to index into the desiredWheelRPM matrix.
     */
    enum WheelRPMIndex
    {
        LF = 0,
        RF = 1,
        LB = 2,
        RB = 3,
    };

    /**
     * Used to index into matrices returned by functions of the form get*Velocity*().
     */
    enum ChassisVelIndex
    {
        X = 0,
        Y = 1,
        R = 2,
    };

    /**
     * Stores the desired RPM of each of the motors in a matrix of the following form:
     * [[leftFront],
     *  [rightFront],
     *  [leftBack],
     *  [rightFront]]
     */
    modm::Matrix<float, 4, 1> desiredWheelRPM;

    tap::algorithms::ExtendedKalman chassisRotationErrorKalman;

    modm::Matrix<float, 3, 4> wheelVelToChassisVelMat;

    float desiredRotation = 0.0f;

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    tap::mock::DjiMotorMock leftFrontMotor;
    tap::mock::DjiMotorMock leftBackMotor;
    tap::mock::DjiMotorMock rightFrontMotor;
    tap::mock::DjiMotorMock rightBackMotor;

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

public:
    ChassisSubsystem(
        aruwsrc::Drivers* drivers,
        tap::motor::MotorId leftFrontMotorId = LEFT_FRONT_MOTOR_ID,
        tap::motor::MotorId leftBackMotorId = LEFT_BACK_MOTOR_ID,
        tap::motor::MotorId rightFrontMotorId = RIGHT_FRONT_MOTOR_ID,
        tap::motor::MotorId rightBackMotorId = RIGHT_BACK_MOTOR_ID,
        tap::gpio::Analog::Pin currentPin = CURRENT_SENSOR_PIN);

    inline int getNumChassisMotors() const override { return 4; }

    inline const tap::motor::DjiMotor* const* getChassisMotorArray() const { return motors; }

    void initialize() override;

    /**
     * Updates the desired wheel RPM based on the passed in x, y, and r components of
     * movement. See the class comment for x and y terminology.
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
     * Run chassis rotation PID on some actual turret angle offset.
     *
     * @param currentAngleError the error as an angle. For autorotation,
     * error between gimbal and center of chassis.
     *
     * @retval a desired rotation speed (wheel speed)
     */
    mockable float chassisSpeedRotationPID(float currentAngleError);

    void refresh() override;

    /**
     * @return A number between 0 and 1 that is the ratio between the rotationRpm and
     *      the max rotation speed.
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

    /**
     * @return the desired rotation component of the chassis speed controller, in shft RPM.
     */
    mockable float getDesiredRotation() const { return desiredRotation; }

private:
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
