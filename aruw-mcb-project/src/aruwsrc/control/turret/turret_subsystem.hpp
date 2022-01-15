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

#ifndef TURRET_SUBSYSTEM_HPP_
#define TURRET_SUBSYSTEM_HPP_

#include "tap/algorithms/contiguous_float.hpp"
#include "tap/algorithms/linear_interpolation_predictor.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/turret/turret_subsystem_interface.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/motor_interface_mock.hpp"
#else
#include "tap/motor/motor_interface.hpp"
#endif

#include "tap/util_macros.hpp"

#include "aruwsrc/util_macros.hpp"
#include "modm/math/filter/pid.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::turret
{
/**
 * Stores software necessary for interacting with two gimbals that control the pitch and
 * yaw of a turret. Provides a convenient API for other commands to interact with a turret.
 */
class TurretSubsystem : public tap::control::turret::TurretSubsystemInterface
{
public:
    static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS1;
    static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR6;
    static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR5;

    static constexpr float MAX_OUT_6020 = 30'000;

#if defined(ALL_SOLDIERS)
    static constexpr float YAW_START_ANGLE = 90.0f;
    static constexpr float PITCH_START_ANGLE = 90.0f;
    static constexpr float YAW_MIN_ANGLE = 0.0f;
    static constexpr float YAW_MAX_ANGLE = 180.0f;
    static constexpr float PITCH_MIN_ANGLE = 63.0f;
    static constexpr float PITCH_MAX_ANGLE = 125.0f;

#ifdef TARGET_SOLDIER_2021
    static constexpr uint16_t YAW_START_ENCODER_POSITION = 6821;
#else
    static constexpr uint16_t YAW_START_ENCODER_POSITION = 1100;
#endif
    static constexpr uint16_t PITCH_START_ENCODER_POSITION = 4100;

    static constexpr float TURRET_CG_X = 12;
    static constexpr float TURRET_CG_Z = 23;
    static constexpr float GRAVITY_COMPENSATION_SCALAR = 2000.0f;
#elif defined(TARGET_HERO)
    static constexpr float YAW_START_ANGLE = 90.0f;
    static constexpr float PITCH_START_ANGLE = 90.0f;
    static constexpr float YAW_MIN_ANGLE = YAW_START_ANGLE - 70.0f;
    static constexpr float YAW_MAX_ANGLE = YAW_START_ANGLE + 70.0f;
    static constexpr float PITCH_MIN_ANGLE = 65.0f;
    static constexpr float PITCH_MAX_ANGLE = 104.0f;

    static constexpr uint16_t YAW_START_ENCODER_POSITION = 3000;
    static constexpr uint16_t PITCH_START_ENCODER_POSITION = 1418;

    static constexpr float TURRET_CG_X = 0;
    static constexpr float TURRET_CG_Z = 0;
    static constexpr float GRAVITY_COMPENSATION_SCALAR = 1.0f;
#elif defined(TARGET_SENTINEL)
    static constexpr float YAW_START_ANGLE = 90.0f;
    static constexpr float YAW_MIN_ANGLE = 5.0f;
    static constexpr float YAW_MAX_ANGLE = 175.0f;
    static constexpr float PITCH_START_ANGLE = 90.0f;
    static constexpr float PITCH_MIN_ANGLE = 15.0f;
    static constexpr float PITCH_MAX_ANGLE = 100.0f;

    static constexpr uint16_t YAW_START_ENCODER_POSITION = 2801;
    static constexpr uint16_t PITCH_START_ENCODER_POSITION = 4150;

    static constexpr float TURRET_CG_X = 0;
    static constexpr float TURRET_CG_Z = 0;
    static constexpr float GRAVITY_COMPENSATION_SCALAR = 1.0f;
#else
    static constexpr float YAW_START_ANGLE = 90.0f;
    static constexpr float YAW_MIN_ANGLE = 0.0f;
    static constexpr float YAW_MAX_ANGLE = 180.0f;
    static constexpr float PITCH_START_ANGLE = 90.0f;
    static constexpr float PITCH_MIN_ANGLE = 0.0f;
    static constexpr float PITCH_MAX_ANGLE = 180.0f;

    static constexpr uint16_t YAW_START_ENCODER_POSITION = 0;
    static constexpr uint16_t PITCH_START_ENCODER_POSITION = 0;

    static constexpr float TURRET_CG_X = 0;
    static constexpr float TURRET_CG_Z = 0;
    static constexpr float GRAVITY_COMPENSATION_SCALAR = 1.0f;
#endif

    /**
     * Constructs a TurretSubsystem.
     *
     * @param[in] drivers Pointer to a drivers singleton object.
     * @param[in] pitchMotor Pointer to pitch motor that this `TurretSubsystem` will own.
     * @param[in] yawMotor Pointer to yaw motor that this `TurretSubsystem` will own.
     * @param[in] limitYaw `true` if the yaw should be limited between `TURRET_YAW_MIN_ANGLE` and
     *      `TURRET_YAW_MAX_ANGLE` and `false` if the yaw should not be limited (if you have a slip
     *      ring).
     * @param[in] chassisFrontBackIdentical `true` if the front and back of the chassis are
     *      interchangable, indicating that `YAW_START_ANGLE` is identical to `YAW_START_ANGLE +
     *      180` in terms of relation to the chassis.
     */
    explicit TurretSubsystem(
        aruwsrc::Drivers* drivers,
        tap::motor::MotorInterface* pitchMotor,
        tap::motor::MotorInterface* yawMotor,
        bool limitYaw = true);

    inline bool yawLimited() const override { return limitYaw; }

    void initialize() override;

    void refresh() override;

    const char* getName() override { return "Turret"; }

    void onHardwareTestStart() override;

    /**
     * @return `true` if both pitch and yaw gimbals are connected.
     */
    inline bool isOnline() const override
    {
        return yawMotor->isMotorOnline() && pitchMotor->isMotorOnline();
    }

    /**
     * @return The wrapped yaw angle of the actual yaw gimbal, in degrees in the chassis frame.
     */
    inline const tap::algorithms::ContiguousFloat& getCurrentYawValue() const override
    {
        return currYawAngle;
    }

    /**
     * @return The wrapped pitch angle of the actual pitch gimbal, in degrees in the chassis frame.
     */
    inline const tap::algorithms::ContiguousFloat& getCurrentPitchValue() const override
    {
        return currPitchAngle;
    }

    /**
     * @return The yaw target as set by the user in `setYawSetpoint`, in the chassis frame.
     */
    inline float getYawSetpoint() const override { return yawTarget.getValue(); }

    /**
     * @return The pitch target as set by the user in `setPitchSetpoint`, in the chassis frame.
     */
    inline float getPitchSetpoint() const override { return pitchTarget.getValue(); }

    /**
     * @return The velocity, in degrees / second, of the turret's pitch yaw, in the chassis frame.
     */
    inline float getYawVelocity() const override { return getVelocity(yawMotor); }

    /**
     * @return The velocity, in degrees / second, of the turret's pitch motor, in the chassis frame.
     */
    inline float getPitchVelocity() const override { return getVelocity(pitchMotor); }

    /**
     * Set a target angle in chassis frame, the angle is accordingly limited.
     * Note that since there is no controller in this subsystem, this target
     * angle merely acts as a safe way to store an angle when using a position controller.
     * The command that contains a controller may use the yaw setpoint as it sees fit.
     */
    void setYawSetpoint(float target) override;

    /**
     * @see setYawSetpoint
     */
    void setPitchSetpoint(float target) override;

    /**
     * @return When `chassisFrontBackIdentical == false`, an angle between [-180, 180] that is the
     *      angle difference of the yaw gimbal from the initial setpoint (`YAW_START_ANGLE`), which
     *      is assumed to be the center of the chassis, in degrees.
     */
    float getYawAngleFromCenter() const override;

    /**
     * @return An angle between [-180, 180] that is the angle difference of the pitch gimbal
     *      from `PITCH_START_ANGLE`, in degrees.
     */
    float getPitchAngleFromCenter() const override;

    /**
     * Attempts to set desired yaw output to the passed in value. If the turret is out of
     * bounds, the output is limited.
     *
     * @param[in] out The desired yaw output, limited to `[-30000, 30000]`.
     */
    mockable void setYawMotorOutput(float out) override;

    /**
     * Attempts to set desired pitch output to the passed in value. If the turret is out of
     * bounds, the output is limited.
     *
     * @param[in] out The desired pitch output, limited to `[-30000, 30000]`.
     */
    mockable void setPitchMotorOutput(float out) override;

    /**
     * Reads the raw pitch and yaw angles and updates the wrapped versions of
     * these angles.
     */
    mockable void updateCurrentTurretAngles();

private:
    tap::algorithms::ContiguousFloat currPitchAngle;
    tap::algorithms::ContiguousFloat currYawAngle;

    uint16_t pitchEncoderWhenLastUpdated;
    uint16_t yawEncoderWhenLastUpdated;

    tap::algorithms::ContiguousFloat yawTarget;
    tap::algorithms::ContiguousFloat pitchTarget;

    bool limitYaw;

    /**
     * @return velocity of 6020 motor, in degrees / sec
     */
    static inline float getVelocity(const tap::motor::MotorInterface* motor)
    {
        return 360 / 60 * motor->getShaftRPM();
    }

    tap::motor::MotorInterface* pitchMotor;
    tap::motor::MotorInterface* yawMotor;
};  // class TurretSubsystem

}  // namespace aruwsrc::control::turret

#endif  // TURRET_SUBSYSTEM_HPP_
