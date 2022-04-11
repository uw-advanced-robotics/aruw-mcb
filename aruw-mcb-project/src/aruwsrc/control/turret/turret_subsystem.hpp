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

#ifndef TURRET_SUBSYSTEM_HPP_
#define TURRET_SUBSYSTEM_HPP_

#include "tap/algorithms/contiguous_float.hpp"
#include "tap/algorithms/linear_interpolation_predictor.hpp"
#include "tap/control/turret_subsystem_interface.hpp"
#include "tap/motor/dji_motor.hpp"

#include "turret_subsystem_config.hpp"

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

namespace aruwsrc::control::turret::algorithms
{
class TurretPitchControllerInterface;
class TurretYawControllerInterface;
}  // namespace aruwsrc::control::turret::algorithms

namespace aruwsrc::control::turret
{
/**
 * Stores software necessary for interacting with two gimbals that control the pitch and
 * yaw of a turret. Provides a convenient API for other commands to interact with a turret.
 *
 * All angles computed using a right hand coordinate system. In other words, yaw is a value from
 * 0-360 rotated counterclockwise when looking at the turret from above. Pitch is a value from 0-360
 * rotated counterclockwise when looking at the turret from the right side of the turret.
 */
class TurretSubsystem : public tap::control::turret::TurretSubsystemInterface
{
public:
    static constexpr float MAX_OUT_6020 = 30'000;

    /**
     * Constructs a TurretSubsystem.
     *
     * @param[in] drivers Pointer to a drivers singleton object.
     * @param[in] pitchMotor Pointer to pitch motor that this `TurretSubsystem` will own.
     * @param[in] yawMotor Pointer to yaw motor that this `TurretSubsystem` will own.
     * @param[in] limitYaw `true` if the yaw should be limited between `YAW_MIN_ANGLE` and
     *      `YAW_MAX_ANGLE` and `false` if the yaw should not be limited (if you have a slip
     *      ring).
     * @param[in] chassisFrontBackIdentical `true` if the front and back of the chassis are
     *      interchangable, indicating that `YAW_START_ANGLE` is identical to `YAW_START_ANGLE +
     *      180` in terms of relation to the chassis.
     */
    explicit TurretSubsystem(
        aruwsrc::Drivers* drivers,
        tap::motor::MotorInterface* pitchMotor,
        tap::motor::MotorInterface* yawMotor,
        const TurretSubsystemConfig& turretConfig);

    mockable inline const TurretSubsystemConfig& getTurretConfig() const { return turretConfig; }

    /// Kept for backwards compatibility reasons, in reality should use use getTurretConfig from now
    /// on
    inline bool yawLimited() const override { return turretConfig.limitYaw; }

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
    inline float getMeasuredYawValue() const override
    {
        return turretConfig.limitYaw ? currYawAngle : currYawAngleWrapped.getValue();
    }

    /**
     * @return The wrapped pitch angle of the actual pitch gimbal, in degrees in the chassis frame.
     */
    inline float getMeasuredPitchValue() const override { return currPitchAngle; }

    float getYawMeasuredSetpointDifference() const
    {
        return turretConfig.limitYaw ? currYawAngleWrapped.difference(yawSetpointWrapped)
                                     : yawSetpoint - currYawAngle;
    }
    float getPitchMeasuredSetpointDifference() const { return pitchSetpoint - currPitchAngle; }

    /**
     * @return The yaw target as set by the user in `setYawSetpoint`, in the chassis frame.
     */
    inline float getYawSetpoint() const override
    {
        return turretConfig.limitYaw ? yawSetpoint : yawSetpointWrapped.getValue();
    }

    /**
     * @return The pitch target as set by the user in `setPitchSetpoint`, in the chassis frame.
     */
    inline float getPitchSetpoint() const override { return pitchSetpoint; }

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

    mockable algorithms::TurretPitchControllerInterface* getPrevRanPitchTurretController() const
    {
        return prevRanPitchTurretController;
    }
    mockable algorithms::TurretYawControllerInterface* getPrevRanYawTurretController() const
    {
        return prevRanYawTurretController;
    }

    mockable void setPrevRanPitchTurretController(
        algorithms::TurretPitchControllerInterface* controller)
    {
        prevRanPitchTurretController = controller;
    }
    mockable void setPrevRanYawTurretController(
        algorithms::TurretYawControllerInterface* controller)
    {
        prevRanYawTurretController = controller;
    }

protected:
    Drivers* drivers;

private:
    const TurretSubsystemConfig turretConfig;

    /// If the turret doesn't have a slip ring (limitVal == false), this value is used instead of
    /// `yawSetpointWrapped`
    float yawSetpoint;
    /// Used when limitValue == true
    tap::algorithms::ContiguousFloat yawSetpointWrapped;
    /// Used when limitYaw == false
    float currYawAngle;
    /// Used when limitYaw == true
    tap::algorithms::ContiguousFloat currYawAngleWrapped;

    float pitchSetpoint;
    float currPitchAngle;

    uint16_t pitchEncoderWhenLastUpdated;
    uint16_t yawEncoderWhenLastUpdated;

    /**
     * @return velocity of 6020 motor, in degrees / sec
     */
    static inline float getVelocity(const tap::motor::MotorInterface* motor)
    {
        return 360 / 60 * motor->getShaftRPM();
    }

    algorithms::TurretPitchControllerInterface* prevRanPitchTurretController = nullptr;
    algorithms::TurretYawControllerInterface* prevRanYawTurretController = nullptr;

    tap::motor::MotorInterface* pitchMotor;
    tap::motor::MotorInterface* yawMotor;
};  // class TurretSubsystem

}  // namespace aruwsrc::control::turret

#endif  // TURRET_SUBSYSTEM_HPP_
