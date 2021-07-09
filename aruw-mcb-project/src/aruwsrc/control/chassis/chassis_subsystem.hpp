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

#include "aruwlib/algorithms/extended_kalman.hpp"
#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/algorithms/smooth_pid.hpp"
#include "aruwlib/communication/gpio/analog.hpp"
#include "aruwlib/control/chassis/chassis_subsystem_interface.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "aruwlib/mock/dji_motor_mock.hpp"
#else
#include "aruwlib/motor/dji_motor.hpp"
#endif

#include "aruwlib/control/chassis/power_limiter.hpp"
#include "aruwlib/motor/m3508_constants.hpp"
#include "aruwlib/util_macros.hpp"

#include "modm/math/filter/pid.hpp"
#include "modm/math/matrix.hpp"

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
class ChassisSubsystem : public aruwlib::control::chassis::ChassisSubsystemInterface
{
public:
    struct ChassisMechanicalConstants
    {
        float motorGearboxRatio;    /// Gearbox ratio of chassis motors.
        float widthBetweenWheelsX;  /// Distance from center of the front and rear wheels (m).
        float widthBetweenWheelsY;  /// Distance from center of the two front wheels (m).
        float wheelRadius;          /// The radius of the chassis wheels (m).
        float gimbalXOffset;  /// Gimbal offset from the center of the chassis, see note above for
                              /// explanation of x and y (m).
        float gimbalYOffset;  /// @see gimbalXOffset
    };

    /**
     * @brief Constructs a ChassisSubsystem
     *
     * @param[in] drivers Pointer to a drivers singleton object.
     * @param[in] mechanicalConstants Mechanical constants of the chassis.
     * @param[in] velocityPidConfig Velocity constants for the chassis PID controllers.
     * @param[in] powerLimiterConfig Power limiting constants.
     * @param[in] maxWheelSpeedSingleMotor Max wheel speed, measured in RPM of the encoder (rather
     *      than shaft) we use this for wheel speed since this is how dji's motors measures motor
     *      speed.
     * @param[in] chassisRevolvePidMaxP Max proportional used for chassis rotation PID controller.
     * @param[in] chassisRevolvePidMaxD Max derivative used for chassis rotation PID controller.
     * @param[in] chassisRevolvePidKD Derivative term used for the chassis rotation PID controller
     *      (see description of controller below).
     * @param[in] chassisRevolvePidMaxOutput The maximum output allowed out of the rotation PID
     *      controller.
     * @param[in] minErrorRotationD The maximum revolve error before we start using the
     *      derivative term.
     * @param[in] minRotationThreshold
     * @param[in] canBus The can bus that the chassis is connected to.
     * @param[in] leftFrontMotorId LF motor id.
     * @param[in] leftBackMotorId LB motor id.
     * @param[in] rightFrontMotorId RF motor id.
     * @param[in] rightBackMotorId RB motor id.
     * @param[in] currentPin The analog input pin that the chassis current sensor is connected to.
     *
     * @note Description of velocity PID controller:
     * First runs kalman filter on the input angle error. All the error calculations in the
     * controller uses this kalman filtered gain.
     * - Next, calculates the proportional term using the kalman filtered angle. Also uses kalman
     *   filtered angle and previous kalman filtered angle for the derivative term; however, the
     *   derivative term will be calculated only if the filtered angle is greater than
     *   `MIN_ERROR_ROTATION_D`.
     * - The wheel speed is calculated by then adding p and d terms and clamping the output to
     *   `MAX_WHEEL_SPEED_SINGLE_MOTOR`.
     */
    ChassisSubsystem(
        aruwlib::Drivers* drivers,
        const ChassisMechanicalConstants& mechanicalConstants,
        const aruwlib::algorithms::PidConfigStruct& velocityPidConfig,
        const aruwlib::control::chassis::PowerLimiterConfig& powerLimiterConfig,
        float maxWheelSpeedSingleMotor,
        float chassisRevolvePidMaxP,
        float chassisRevolvePidMaxD,
        float chassisRevolvePidKD,
        float chassisRevolvePidMaxOutput,
        float minErrorRotationD,
        float minRotationThreshold,
        aruwlib::can::CanBus canBus,
        aruwlib::motor::MotorId leftFrontMotorId,
        aruwlib::motor::MotorId leftBackMotorId,
        aruwlib::motor::MotorId rightFrontMotorId,
        aruwlib::motor::MotorId rightBackMotorId,
        aruwlib::gpio::Analog::Pin currentPin);

    inline int getNumChassisMotors() const override { return 4; }

    inline const aruwlib::motor::DjiMotor* const* getChassisMotorArray() const { return motors; }

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
     * error between gimbal and center of chassis. See description of the
     * controller above.
     * @param kp[in] proportional gain for PID caluclation
     *
     * @retval a desired rotation speed (wheel speed)
     */
    mockable float chassisSpeedRotationPID(float currentAngleError, float kp);

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

    inline float getMaxWheelSpeedSingleMotor() const { return MAX_WHEEL_SPEED_SINGLE_MOTOR; }

    void onHardwareTestStart() override;

    /**
     * @return the desired rotation component of the chassis speed controller, in shft RPM.
     */
    mockable float getDesiredRotation() const { return desiredRotation; }

private:
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

    const float MAX_WHEEL_SPEED_SINGLE_MOTOR;

    const float MOTOR_GEARBOX_RATIO;
    const float WIDTH_BETWEEN_WHEELS_X;
    const float WIDTH_BETWEEN_WHEELS_Y;
    const float WHEEL_RADIUS;
    const float GIMBAL_X_OFFSET;
    const float GIMBAL_Y_OFFSET;

    const float CHASSIS_REVOLVE_PID_MAX_P;
    const float CHASSIS_REVOLVE_PID_MAX_D;
    const float CHASSIS_REVOLVE_PID_KD;
    const float CHASSIS_REVOLVE_PID_MAX_OUTPUT;
    const float MIN_ERROR_ROTATION_D;

    const float MIN_ROTATION_THRESHOLD;

    // wheel velocity PID variables
    modm::Pid<float> leftFrontVelocityPid;
    modm::Pid<float> leftBackVelocityPid;
    modm::Pid<float> rightFrontVelocityPid;
    modm::Pid<float> rightBackVelocityPid;

    /**
     * Stores the desired RPM of each of the motors in a matrix of the following form:
     * [[leftFront],
     *  [rightFront],
     *  [leftBack],
     *  [rightFront]]
     */
    modm::Matrix<float, 4, 1> desiredWheelRPM;

    aruwlib::algorithms::ExtendedKalman chassisRotationErrorKalman;

    modm::Matrix<float, 3, 4> wheelVelToChassisVelMat;

    float desiredRotation = 0.0f;

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

    aruwlib::motor::DjiMotor* motors[4];
    aruwlib::control::chassis::PowerLimiter chassisPowerLimiter;
    const aruwlib::motor::M3508Constants motorConstants;

    /**
     * When you input desired x, y, an r values, this function translates
     * and sets the RPM of individual chassis motors.
     */
    void mecanumDriveCalculate(float x, float y, float r, float maxWheelSpeed);

    void updateMotorRpmPid(
        modm::Pid<float>* pid,
        aruwlib::motor::DjiMotor* const motor,
        float desiredRpm);

    /**
     * Converts the velocity matrix from raw RPM to wheel velocity in m/s.
     */
    inline modm::Matrix<float, 4, 1> convertRawRPM(const modm::Matrix<float, 4, 1>& mat) const
    {
        const float ratio = 2.0f * aruwlib::algorithms::PI * MOTOR_GEARBOX_RATIO / 60.0f;
        return mat * ratio;
    }
};  // class ChassisSubsystem

}  // namespace chassis

}  // namespace aruwsrc

#endif  // CHASSIS_SUBSYSTEM_HPP_
