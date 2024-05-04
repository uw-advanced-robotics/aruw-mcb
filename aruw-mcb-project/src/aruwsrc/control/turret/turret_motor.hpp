/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TURRET_MOTOR_HPP_
#define TURRET_MOTOR_HPP_

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/motor/motor_interface.hpp"
#include "tap/util_macros.hpp"

#include "algorithms/turret_controller_interface.hpp"
#include "modm/math/geometry/angle.hpp"

#include "turret_motor_config.hpp"

namespace aruwsrc::control::turret
{
/**
 * Logic encapsulating the control of a single axis of a turret gimbal motor. Contains logic for
 * storing chassis relative position measurements and setpoints and logic for limiting the angle
 * setpoint.
 *
 * Currently, there are GM6020-specific motor parameters in this object such that it is expected
 * that the gimbal motor used is a 6020, but in general with some taproot-side MRs, this class can
 * be generalized to work with any motor interface.
 */
class TurretMotor
{
public:
    /// Maximum output, voltage control between [-24, 24] volts scaled up to [-30,000, 30,000] units
    static constexpr float MAX_OUT_6020 = 30'000;

    /**
     * Construct a turret motor with some particular hardware motor interface and a motor
     * configuration struct.
     */
    TurretMotor(tap::motor::MotorInterface *motor, const TurretMotorConfig &motorConfig);

    mockable inline void initialize() { motor->initialize(); }

    /// Updates the measured motor angle
    mockable void updateMotorAngle();

    /**
     * Set the motor's desired output when the motor is online. The output is expected to be in the
     * motor's unitless form. For the GM6020, the motor output is limited between [-MAX_OUT_6020,
     * MAX_OUT_6020].
     *
     * @param[in] out The desired motor output.
     */
    mockable void setMotorOutput(float out);

    /**
     * Attaches the specified turretController to this turret motor. This does not give ownership
     * of the controller to this object. Instead it allows commands to know which turret controller
     * is currently being run (since turret controllers are shared by commands but persist across
     * different commands).
     */
    mockable inline void attachTurretController(
        const algorithms::TurretControllerInterface *turretController)
    {
        this->turretController = turretController;
    }

    /**
     * Sets (and limits!) the chassis frame turret measurement.
     *
     * The setpoint is limited between the min and max config angles as specified in the
     * constructor.
     */
    mockable void setChassisFrameSetpoint(float setpoint);

    /// @return `true` if the hardware motor is connected and powered on
    mockable inline bool isOnline() const { return motor->isMotorOnline(); }

    /**
     * @return turret motor angle setpoint relative to the chassis, in radians, not normalized
     */
    mockable inline float getChassisFrameSetpoint() const { return chassisFrameSetpoint; }

    /// @return turret motor angle measurement relative to the chassis, in radians, wrapped between
    /// [0, 2 PI)
    mockable inline const tap::algorithms::WrappedFloat &getChassisFrameMeasuredAngle() const
    {
        return chassisFrameMeasuredAngle;
    }

    /// @return turret motor angle measurement in chassis frame, unwrapped (not normalized).
    mockable inline float getChassisFrameUnwrappedMeasuredAngle() const
    {
        return chassisFrameUnwrappedMeasurement;
    }

    /**
     * @return angular velocity of the turret, in rad/sec, positive rotation is defined by the
     * motor.
     */
    mockable inline float getChassisFrameVelocity() const
    {
        return (M_TWOPI / 60) * motor->getShaftRPM();
    }

    /**
     * @return A normalized angle between [-PI, PI] that is the angle difference between the turret
     * and the turret motors' specified "start angle" (specified upon construction in the
     * TurretMotorConfig struct).
     */
    mockable inline float getAngleFromCenter() const
    {
        return tap::algorithms::WrappedFloat(
                   chassisFrameMeasuredAngle.getWrappedValue() - config.startAngle,
                   -M_PI,
                   M_PI)
            .getWrappedValue();
    }

    /// @return turret controller controlling this motor (as specified by `attachTurretController`)
    mockable const algorithms::TurretControllerInterface *getTurretController() const
    {
        return turretController;
    }

    /// @return The turret motor config struct associated with this motor
    mockable const TurretMotorConfig &getConfig() const { return config; }

    /**
     * @return Valid minimum error between the chassis relative setpoint and measurement, in
     * radians.
     *
     * @note A valid measurement error is either:
     * - The shortest wrapped distance between the chassis frame measurement and setpoint
     *   if the turret motor is not limited to some min/max values.
     * - The absolute difference between the chassis frame measurement and setpoint if the
     *   turret motor is limited to some min/max values.
     */
    mockable float getValidChassisMeasurementError() const;

    /**
     * Same as getValidChassisMeasurementError, but operates on "wrapped" angles regardless of
     * turret limit status.
     * This function should be used for checking tolerances, while
     * getValidChassisMeasurementErrorWrapped should be used for PID controllers and other
     * applications that require directionality.
     */
    float getValidChassisMeasurementErrorWrapped() const;

    /**
     * @param[in] measurement A turret measurement in the chassis frame, an angle in radians. This
     * can be encoder based (via getChassisFrameMeasuredAngle) or can be measured by some other
     * means (for example, an IMU on the turret that is than transformed to the chassis frame).
     *
     * @return The minimum error between the chassis frame setpoint and the specified measurement.
     * If the turret motor is not limited, the error is wrapped between [0, 2*PI), otherwise the
     * error is absolute.
     *
     * @note Call getValidChassisMeasurementError if you want the error between the chassis-frame
     * setpoint and measurement
     *
     * @note The measurement does not need to be normalized to [0, 2*PI]. In fact, if the turret
     * motor is limited, an unwrapped measurement should be used in order to avoid unexpected
     * wrapping errors.
     *
     * @note Before calling this function, you **must** first set the chassis frame setpoint before
     * calling this function (i.e. call `setChassisFrameSetpoint`).
     */
    mockable float getValidMinError(const float setpoint, const float measurement) const;

    /**
     * "Unwraps" a normalized (between [0, 2PI)) angle. Does so in such a way that setpoint returned
     * is an equivalent angle to the specified setpoint, but the setpoint returned is the closest
     * possible angle to the passed in measurement.
     *
     * @param[in] measurement Some non-normalized measurement in radians. The returned setpoint will
     * be the closest possible angle within the limits of this turret motor to this measurement.
     * @param[in] setpoint A setpoint in radians that is not necessarily normalized.
     *
     * @return A setpoint angle in radians that is unwrapped and is the closest value between the
     * min/max angle values that is closest to the measurement.
     */
    static float getClosestNonNormalizedSetpointToMeasurement(float measurement, float setpoint);

    /**
     * Translates the setpoint that may or may not be within the range of the turret to an angle
     * that is within the min/max bounds of the turret motor if possible. If there is no valid
     * setpoint within the min/max bounds, this function will return the original setpoint.
     *
     * For example, if the minimum angle is -PI and the max angle is PI, if the setpoint is -2*PI
     * then the value returned is -2*PI + 2*PI = 0. This angle is within the acceptable bounds and
     * rotationally equivalent to the specified setpoint.
     *
     * @param[in] setpoint Some non-normalized turret setpoint, in radians.
     *
     * @return The translated value.
     */
    float getSetpointWithinTurretRange(float setpoint) const;

    int16_t getMotorOutput() const { return motor->getOutputDesired(); }

    /**
     * Takes in a normalized setpoint and "unnormalizes" it. Finds an equivalent unwrapped setpoint
     * that is closest to the TurretMotor's measured angle that is also within bounds of the min/max
     * angles.
     *
     * @note This function assumes that if the TurretMotor's angle is not limited, this function
     * does not need to perform any updates of setpointToUnwrap since the turret controller will
     * normalize both the setpoint and measurement before performing computation.
     *
     * @note A TurretController must be attached to the TurretMotor in order for this function to do
     * anything. If a TurretController is not associated, we don't know how to convert the
     * setpointToUnwrap into the chassis reference frame.
     *
     * @param[in] setpointToUnwrap Setpoint to update, a setpoint angle measurement in radians in
     * the same reference frame as the attached turretController's reference frame.
     * @return The updated setpointToUnwrap, or the same setpointToUnwrap if no updating necessary
     * or if the notes above apply.
     */
    inline float unwrapTargetAngle(float setpointToUnwrap) const
    {
        if (turretController == nullptr || !config.limitMotorAngles)
        {
            return setpointToUnwrap;
        }

        setpointToUnwrap = getClosestNonNormalizedSetpointToMeasurement(
            turretController->getMeasurement(),
            setpointToUnwrap);

        setpointToUnwrap =
            turretController->convertChassisAngleToControllerFrame(getSetpointWithinTurretRange(
                turretController->convertControllerAngleToChassisFrame(setpointToUnwrap)));

        return setpointToUnwrap;
    }

private:
    const TurretMotorConfig config;

    /// Low-level motor object that this object interacts with
    tap::motor::MotorInterface *motor;

    /// Associated turret controller interface that is being used by a command to control this motor
    const algorithms::TurretControllerInterface *turretController = nullptr;

    /**
     * Offset applied when the motor is turned on. When the turret is turned on, the distance
     * between the start encoder value and the current encoder value is measured. If the magnitude
     * of this difference is greater than DjiMotor::ENC_RESOLUTION / 2, an offset of
     * DjiMotor::ENC_RESOLUTION is applied to measured encoder values to avoid bad angle wrapping.
     *
     * If equal to INT16_MIN, needs to be re-computed
     */
    int16_t startEncoderOffset = INT16_MIN;

    /// Unwrapped chassis frame setpoint specified by the user and limited to `[config.minAngle,
    /// config.maxAngle]`. Units radians.
    float chassisFrameSetpoint;

    /// Wrapped chassis frame measured angle between [0, 2*PI). Units radians.
    tap::algorithms::WrappedFloat chassisFrameMeasuredAngle;

    /// Unwrapped chassis frame measured angle. Units radians.
    float chassisFrameUnwrappedMeasurement;

    int64_t lastUpdatedEncoderValue;
};
}  // namespace aruwsrc::control::turret

#endif  // TURRET_MOTOR_HPP_
