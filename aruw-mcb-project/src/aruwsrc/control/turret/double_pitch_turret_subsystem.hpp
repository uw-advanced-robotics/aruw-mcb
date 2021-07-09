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

#ifndef SENTINEL_TURRET_SUBSYSTEM_HPP_
#define SENTINEL_TURRET_SUBSYSTEM_HPP_

#include <modm/math/filter/pid.hpp>

#include "aruwlib/algorithms/contiguous_float.hpp"
#include "aruwlib/algorithms/linear_interpolation.hpp"
#include "aruwlib/algorithms/smooth_pid.hpp"
#include "aruwlib/control/turret/turret_subsystem_interface.hpp"
#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "aruwlib/mock/dji_motor_mock.hpp"
#else
#include "aruwlib/motor/dji_motor.hpp"
#endif

namespace aruwsrc::control::turret
{
/**
 * Stores software necessary for interacting with two gimbals that control the pitch and
 * yaw of a turret. Provides a convenient API for other commands to interact with a turret.
 */
class DoublePitchTurretSubsystem : public aruwlib::control::turret::TurretSubsystemInterface
{
public:
    /**
     * @param[in] drivers Pointer to a drivers singleton object.
     * @param[in] yawPidConfig PID configuration for yaw controller.
     * @param[in] pitchPidConfig PID configuration for pitch controller.
     * @param[in] userYawInputScalar Scaler to multiply user input by.
     * @param[in] userPitchInputScalar @see userYawInputScalar.
     * @param[in] pitchGravityCompensationKp Gravity compensation proportional gain.
     * @param[in] yawStartAngle The angle that the yaw motor starts at.
     * @param[in] pitchStartAngle The angle that the pitch motor starts at.
     * @param[in] yawMinAngle Minimum yaw angle that the turret is limited to (in degrees).
     * @param[in] yawMaxAngle Maximum yaw angle that the turret is limited to (in degrees).
     * @param[in] pitchMinAngle Minimum pitch angle that the turret is limited to (in degrees).
     * @param[in] pitchMaxAngle Maximum pitch angle that the turret is limited to (in degrees).
     * @param[in] yawStartEncoderPosition The yaw encoder value that is associated with the
     *      `startAngle`.
     * @param[in] pitch90DegEncoderPositionLeft The left pitch encoder value that is associated with
     *      90 degrees.
     * @param[in] pitch90DegEncoderPositionRight The right pitch encoder value that is associated
     *      with 90 degrees.
     * @param[in] motorCanBus Can bus that the turret is connected to.
     * @param[in] pitchMotorLeftId, pitchMotorRightId DJI motor id for pitch motor.
     * @param[in] yawMotorId DJI motor id for yaw motor.
     * @param[in] limitYaw `true` if the yaw should be limited between `TURRET_YAW_MIN_ANGLE` and
     *      `TURRET_YAW_MAX_ANGLE` and `false` if the yaw should not be limited (if you have a slip
     *      ring).
     */
    explicit DoublePitchTurretSubsystem(
        aruwlib::Drivers* drivers,
        const aruwlib::algorithms::PidConfigStruct& yawPidConfig,
        const aruwlib::algorithms::PidConfigStruct& pitchPidConfig,
        float userYawInputScalar,
        float userPitchInputScalar,
        float pitchGravityCompensationKp,
        float yawStartAngle,
        float pitchStartAngle,
        float yawMinAngle,
        float yawMaxAngle,
        float pitchMinAngle,
        float pitchMaxAngle,
        float yawStartEncoderPosition,
        float pitch90DegEncoderPositionLeft,
        float pitch90DegEncoderPositionRight,
        aruwlib::can::CanBus motorCanBus,
        aruwlib::motor::MotorId pitchMotorRightId,
        aruwlib::motor::MotorId pitchMotorLeftId,
        aruwlib::motor::MotorId yawMotorId,
        bool limitYaw = true);

    void initialize() override;

    void refresh() override;

    /**
     * @return The yaw target as set by the user in `setYawSetpoint`.
     */
    inline float getYawSetpoint() const override { return yawTarget.getValue(); }

    /**
     * @return The pitch target as set by the user in `setPitchSetpoint`.
     */
    inline float getPitchSetpoint() const override { return pitchTarget.getValue(); }

    /**
     * Set a target angle in chassis frame, the angle is accordingly limited.
     * Note that since there is no controller in this subsystem, this target
     * angle merely acts as a safe way to set angles when using a position controller.
     */
    void setYawSetpoint(float target) override;

    /**
     * @see setYawSetpoint
     */
    void setPitchSetpoint(float target) override;

    /**
     * @return The wrapped yaw angle of the actual yaw gimbal.
     */
    const aruwlib::algorithms::ContiguousFloat& getCurrentYawValue() const override;

    /**
     * @see getCurrentYawValue.
     */
    const aruwlib::algorithms::ContiguousFloat& getCurrentPitchValue() const override;

    /**
     * @return `true` if both pitch and yaw gimbals are connected.
     */
    inline bool isOnline() const override
    {
        return yawMotor.isMotorOnline() && pitchMotorLeft.isMotorOnline() &&
               pitchMotorRight.isMotorOnline();
    }

    /**
     * @return the yaw velocity of the turret, in degrees / second
     */
    inline float getYawVelocity() const override { return getVelocity(yawMotor); }

    /**
     * @see getYawVelocity.
     */
    inline float getPitchVelocity() const override
    {
        return (getVelocity(pitchMotorLeft) + getVelocity(pitchMotorRight)) / 2.0f;
    }

    /**
     * @return An angle between [-180, 180] that is the angle difference of the yaw gimbal
     *      from center (90 degrees), in degrees.
     */
    float getYawAngleFromCenter() const override;

    /**
     * @return An angle between [-180, 180] that is the angle difference of the pitch gimbal
     *      from center (90 degrees), in degrees.
     */
    float getPitchAngleFromCenter() const override;

    const char* getName() override { return "Sentinel Turret"; }

private:
    const float TURRET_YAW_START_ANGLE;
    const float TURRET_YAW_MIN_ANGLE;
    const float TURRET_YAW_MAX_ANGLE;
    const float TURRET_PITCH_START_ANGLE;
    const float TURRET_PITCH_MIN_ANGLE;
    const float TURRET_PITCH_MAX_ANGLE;
    const float USER_YAW_INPUT_SCALAR;
    const float USER_PITCH_INPUT_SCALAR;
    const float PITCH_GRAVITY_COMPENSATION_KP;
    const uint16_t YAW_START_ENCODER_POSITION;
    const uint16_t PITCH_90DEG_ENCODER_POSITION_LEFT;
    const uint16_t PITCH_90DEG_ENCODER_POSITION_RIGHT;

    aruwlib::algorithms::ContiguousFloat currLeftPitchAngle;
    aruwlib::algorithms::ContiguousFloat currRightPitchAngle;
    aruwlib::algorithms::ContiguousFloat currYawAngle;

    aruwlib::algorithms::ContiguousFloat yawTarget;
    aruwlib::algorithms::ContiguousFloat pitchTarget;

    aruwlib::algorithms::SmoothPid yawMotorPid;
    aruwlib::algorithms::SmoothPid leftPitchPid;
    aruwlib::algorithms::SmoothPid rightPitchPid;

    uint32_t prevTime;

    bool limitYaw;

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    aruwlib::mock::DjiMotorMock pitchMotorLeft;
    aruwlib::mock::DjiMotorMock pitchMotorRight;
    aruwlib::mock::DjiMotorMock yawMotor;

private:
#else
    aruwlib::motor::DjiMotor pitchMotorLeft;
    aruwlib::motor::DjiMotor pitchMotorRight;
    aruwlib::motor::DjiMotor yawMotor;
#endif

    /**
     * Reads the raw pitch and yaw angles and updates the wrapped versions of
     * these angles.
     */
    void updateCurrentTurretAngles();

    /**
     * Update the angle of the specified motor.
     *
     * @param[in] motor The motor whose angle to read.
     * @param[in] calibrationEncoderValue The encoder value associated with the calibration angle.
     * @param[in] calibrationAngle The angle that corresponds to the calibrationEncoderValue. So if
     *      the calibrationEncoderValue is 1000 and the calibrationAngle is 90, when the motor's
     *      encoder reads 1000, the angle will be 90 an all other motor angles will be associated on
     *      this relationship.
     * @param[out] turretAngle The wrapped angle to update.
     */
    static void updateTurretAngle(
        const aruwlib::motor::DjiMotor& motor,
        uint16_t calibrationEncoderValue,
        float calibrationAngle,
        aruwlib::algorithms::ContiguousFloat& turretAngle);

    /**
     * Runs the passed in motor's PID controller given the specified motor motor,
     * angle, and other controller information.
     *
     * @param[in] currAngle The current wrapped turret angle associated with the motor.
     * @param[in] setpoint The setpoint used to compare with the currAngle when determining the
     *      desired motor output.
     * @param[in] dt The delta time between the current and previous calls to refresh.
     * @param[in] errorBtwnMotors
     * @param[in] pitchGravityCompensation
     * @param[out] pidController The PID controller to use and update.
     * @param[out] motor The motor to set the output on.
     */
    static void runPositionPid(
        const aruwlib::algorithms::ContiguousFloat& currAngle,
        const aruwlib::algorithms::ContiguousFloat& setpoint,
        const uint32_t dt,
        const float errorBtwnMotors,
        const float pitchGravityCompensation,
        aruwlib::algorithms::SmoothPid& pidController,
        aruwlib::motor::DjiMotor& motor);

    /**
     * Attempts to set desired passed in motor output to the passed in motor. If the turret is out
     * of bounds, the output is limited.
     *
     * @param[in] out The desired output, limited to `[-30000, 30000]`.
     * @param[in] currAngle The current motor angle, in degrees
     * @param[in] motor Reference to the motor to set.
     */
    static void setMotorOutput(float out, aruwlib::motor::DjiMotor& motor);

    /**
     * @return the angular velocity of a motor, in degrees / second, of the passed in motor. 360
     *      degrees / (60 seconds / minute) * (shaftRPM in shaft rotation / minute).
     */
    static inline int32_t getVelocity(const aruwlib::motor::DjiMotor& motor)
    {
        return 360 / 60 * static_cast<int32_t>(motor.getShaftRPM());
    }
};  // class DoublePitchTurretSubsystem

}  // namespace aruwsrc::control::turret

#endif  // SENTINEL_TURRET_SUBSYSTEM_HPP_
