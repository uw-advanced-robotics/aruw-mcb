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

#include <aruwlib/algorithms/extended_kalman.hpp>
#include <aruwlib/control/subsystem.hpp>

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include <aruwlib/mock/DJIMotorMock.hpp>
#else
#include <aruwlib/motor/dji_motor.hpp>
#endif

#include <modm/math/filter/pid.hpp>

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
class ChassisSubsystem : public aruwlib::control::Subsystem
{
public:
    /**
     * Max wheel speed, measured in RPM of the encoder (rather than shaft)
     * we use this for wheel speed since this is how dji's motors measures motor speed.
     */
    static const int MAX_WHEEL_SPEED_SINGLE_MOTOR = 7000;

    /**
     * The minimum desired wheel speed for chassis rotation, measured in RPM before
     * we start slowing down translational speed.
     */
    static constexpr float MIN_ROTATION_THRESHOLD = 800.0f;

private:
#if defined(TARGET_SOLDIER) || defined(TARGET_OLD_SOLDIER)
    /**
     * Velocity PID gains and constants.
     */
    const float VELOCITY_PID_KP = 20.0f;
    const float VELOCITY_PID_KI = 0.0f;
    const float VELOCITY_PID_KD = 0.0f;
    const float VELOCITY_PID_MAX_ERROR_SUM = 0.0f;
    /**
     * This max output is measured in the c620 robomaster translated current.
     * Per the datasheet, the controllable current range is -16384 ~ 0 ~ 16384.
     * The corresponding speed controller output torque current range is
     * -20 ~ 0 ~ 20 A.
     */
    static constexpr float VELOCITY_PID_MAX_OUTPUT = 16000.0f;

    /**
     * Rotation PID:
     * A PD controller for chassis autorotation PID that runs on error between
     * chassis rotation error.
     *
     * Description of controller:
     * - First runs kalman filter on the input angle error. All the error calculations in
     *   the controller uses this kalman filtered gain.
     * - Next, calculates the proportional term using the kalman filtered angle.
     *   Also uses kalman filtered angle and previous kalman filtered angle for the
     *   derivative term; however, the derivative term will be calculated only if the
     *   filtered angle is greater than `MIN_ERROR_ROTATION_D`.
     * - The wheel speed is calculated by then adding p and d terms and clamping the output
     *   to `MAX_WHEEL_SPEED_SINGLE_MOTOR`.
     *
     * The P gain is specified by the user and thus is not specified below.
     */
    static constexpr float CHASSIS_REVOLVE_PID_MAX_P = MAX_WHEEL_SPEED_SINGLE_MOTOR;
    /**
     * Derivative term used in chassis PID.
     */
    static constexpr float CHASSIS_REVOLVE_PID_KD = 500.0f;
    /**
     * Derivative max term.
     */
    static constexpr float CHASSIS_REVOLVE_PID_MAX_D = 0.0f;
    /**
     * The maximum revolve error before we start using the derivative term.
     */
    static const int MIN_ERROR_ROTATION_D = 0;

    // mechanical chassis constants, all in mm
    /**
     * Radius of the wheels (mm).
     */
    static constexpr float WHEEL_RADIUS = 76.0f;
    /**
     * Distance from center of the two front wheels (mm).
     */
    static constexpr float WIDTH_BETWEEN_WHEELS_Y = 366.0f;
    /**
     * Distance from center of the front and rear wheels (mm).
     */
    static constexpr float WIDTH_BETWEEN_WHEELS_X = 366.0f;
    /**
     * Gimbal offset from the center of the chassis, see note above for explanation of x and y (mm).
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
    static constexpr float VELOCITY_PID_KP = 0.0f;
    static constexpr float VELOCITY_PID_KI = 0.0f;
    static constexpr float VELOCITY_PID_KD = 0.0f;
    static constexpr float VELOCITY_PID_MAX_ERROR_SUM = 0.0f;
    static constexpr float VELOCITY_PID_MAX_OUTPUT = 0.0f;

    /**
     * Rotation PID gains and constants.
     * No i, max error sum the same as `MAX_WHEEL_SPEED_SINGLE_MOTOR`, proportional
     * gain specified by user.
     */
    static constexpr float CHASSIS_REVOLVE_PID_MAX_P = 0.0;
    /**
     * Derivative term used in chassis PID.
     */
    static constexpr float CHASSIS_REVOLVE_PID_KD = 0.0;
    /**
     * The maximum revolve error before we start using the derivative term.
     */
    static const int MIN_ERROR_ROTATION_D = 0;
    /**
     * Derivative max term.
     */
    static constexpr float CHASSIS_REVOLVE_PID_MAX_D = 0.0f;
    // mechanical chassis constants
    /**
     * Radius of the wheels.
     */
    static constexpr float WHEEL_RADIUS = 76.0f;
    /**
     * Distance from center of the two front wheels.
     */
    static constexpr float WIDTH_BETWEEN_WHEELS_Y = 517.0f;
    /**
     * Distance from center of the front and rear wheels.
     */
    static constexpr float WIDTH_BETWEEN_WHEELS_X = 600.0f;
    /**
     * Gimbal offset from the center of the chassis, see note above for explanation of x and y.
     */
    static constexpr float GIMBAL_X_OFFSET = 175.0f;
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
     * Rotation PID gains and constants.
     * No i, max error sum the same as `MAX_WHEEL_SPEED_SINGLE_MOTOR`, proportional
     * gain specified by user.
     */
    static constexpr float CHASSIS_REVOLVE_PID_MAX_P = 0.0;
    /**
     * Derivative term used in chassis PID.
     */
    static constexpr float CHASSIS_REVOLVE_PID_KD = 0.0;
    /**
     * Derivative max term.
     */
    static constexpr float CHASSIS_REVOLVE_PID_MAX_D = 0.0f;
    /**
     * The maximum revolve error before we start using the derivative term.
     */
    static const int MIN_ERROR_ROTATION_D = 0;

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
    static constexpr aruwlib::motor::MotorId LEFT_FRONT_MOTOR_ID = aruwlib::motor::MOTOR2;
    static constexpr aruwlib::motor::MotorId LEFT_BACK_MOTOR_ID = aruwlib::motor::MOTOR3;
    static constexpr aruwlib::motor::MotorId RIGHT_FRONT_MOTOR_ID = aruwlib::motor::MOTOR1;
    static constexpr aruwlib::motor::MotorId RIGHT_BACK_MOTOR_ID = aruwlib::motor::MOTOR4;

#if defined(TARGET_OLD_SOLDIER)
    static constexpr aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS1;
#else
    static constexpr aruwlib::can::CanBus CAN_BUS_MOTORS = aruwlib::can::CanBus::CAN_BUS2;
#endif

    // wheel velocity PID variables
    modm::Pid<float> leftFrontVelocityPid;
    modm::Pid<float> leftBackVelocityPid;
    modm::Pid<float> rightFrontVelocityPid;
    modm::Pid<float> rightBackVelocityPid;

    // translate all input into a desired wheel RPM when given
    float leftFrontRpm;
    float leftBackRpm;
    float rightFrontRpm;
    float rightBackRpm;

    float chassisDesiredR = 0.0f;

    aruwlib::algorithms::ExtendedKalman chassisRotationErrorKalman;

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    aruwlib::mock::DjiMotorMock leftFrontMotor;
    aruwlib::mock::DjiMotorMock leftBackMotor;
    aruwlib::mock::DjiMotorMock rightFrontMotor;
    aruwlib::mock::DjiMotorMock rightBackMotor;

private:
#else
    // motors
    aruwlib::motor::DjiMotor leftFrontMotor;
    aruwlib::motor::DjiMotor leftBackMotor;
    aruwlib::motor::DjiMotor rightFrontMotor;
    aruwlib::motor::DjiMotor rightBackMotor;
#endif

public:
    ChassisSubsystem(
        aruwlib::Drivers* drivers,
        aruwlib::motor::MotorId leftFrontMotorId = LEFT_FRONT_MOTOR_ID,
        aruwlib::motor::MotorId leftBackMotorId = LEFT_BACK_MOTOR_ID,
        aruwlib::motor::MotorId rightFrontMotorId = RIGHT_FRONT_MOTOR_ID,
        aruwlib::motor::MotorId rightBackMotorId = RIGHT_BACK_MOTOR_ID)
        : aruwlib::control::Subsystem(drivers),
          leftFrontVelocityPid(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT),
          leftBackVelocityPid(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT),
          rightFrontVelocityPid(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT),
          rightBackVelocityPid(
              VELOCITY_PID_KP,
              VELOCITY_PID_KI,
              VELOCITY_PID_KD,
              VELOCITY_PID_MAX_ERROR_SUM,
              VELOCITY_PID_MAX_OUTPUT),
          leftFrontRpm(0),
          leftBackRpm(0),
          rightFrontRpm(0),
          rightBackRpm(0),
          chassisRotationErrorKalman(1.0f, 0.0f),
          leftFrontMotor(
              drivers,
              leftFrontMotorId,
              CAN_BUS_MOTORS,
              false,
              "left front drive motor"),
          leftBackMotor(drivers, leftBackMotorId, CAN_BUS_MOTORS, false, "left back drive motor"),
          rightFrontMotor(
              drivers,
              rightFrontMotorId,
              CAN_BUS_MOTORS,
              false,
              "right front drive motor"),
          rightBackMotor(drivers, rightBackMotorId, CAN_BUS_MOTORS, false, "right back drive motor")
    {
    }

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
    void setDesiredOutput(float x, float y, float r);

    /**
     * Run chassis rotation PID on some actual turret angle offset.
     *
     * @param currentAngleError the error as an angle. For autorotation,
     * error between gimbal and center of chassis. See description of the
     * controller above.
     * @param kp[in] proportional gain for PID caluclation
     *
     * @retval a desired rotation speed (wheel speed)
     */
    float chassisSpeedRotationPID(float currentAngleError, float kp);

    void refresh() override;

    /**
     * @return A number between 0 and 1 that is the ratio between the rotationRpm and
     *      the max rotation speed.
     */
    float calculateRotationTranslationalGain(float chassisRotationDesiredWheelspeed);

    /**
     * @return The desired rotation based on what was input into the subsystem via
     *      `setDesiredOutput`.
     */
    float getChassisDesiredRotation() const;

    void runHardwareTests() override;

    const char* getName() override { return "Chassis"; }

private:
    /**
     * When you input desired x, y, an r values, this function translates
     * and sets the RPM of individual chassis motors.
     */
    void mecanumDriveCalculate(float x, float y, float r, float maxWheelSpeed);

    void updateMotorRpmPid(
        modm::Pid<float>* pid,
        aruwlib::motor::DjiMotor* const motor,
        float desiredRpm);

};  // class ChassisSubsystem

}  // namespace chassis

}  // namespace aruwsrc

#endif  // CHASSIS_SUBSYSTEM_HPP_
