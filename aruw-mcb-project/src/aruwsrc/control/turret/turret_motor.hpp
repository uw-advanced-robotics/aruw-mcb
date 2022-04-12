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

#include "tap/algorithms/contiguous_float.hpp"
#include "tap/motor/motor_interface.hpp"

#include "algorithms/turret_controller_interface.hpp"
#include "modm/math/geometry/angle.hpp"

#include "turret_motor_config.hpp"
#include "tap/util_macros.hpp"

namespace aruwsrc::control::turret
{
class TurretMotor
{
public:
    static constexpr float MAX_OUT_6020 = 30'000;

    TurretMotor(tap::motor::MotorInterface *motor, const TurretMotorConfig &motorConfig);

    mockable inline void initialize() { motor->initialize(); }

    /// Updates the measured motor angle
    mockable void updateMotorAngle();

    mockable void setMotorOutput(float out);

    /**
     * Attaches the specified turretController with the turret motor. This does not give ownership
     * of the controller to this object. Instead it allows commands to know which turret controller
     * is currently being run (since turret controllers are shared by commands but persist across
     * different commands).
     */
    mockable void attachTurretController(const algorithms::TurretControllerInterface *turretController)
    {
        this->turretController = turretController;
    }

    /// Sets (and limits!) the chassis frame turret measurement
    mockable void setChassisFrameSetpoint(float setpoint);

    /// @return `true` if the hardware motor is connected and powered on
    mockable inline bool isOnline() const { return motor->isMotorOnline(); }

    /// @return turret motor angle setpoint relative to the chassis, in radians
    mockable inline const tap::algorithms::ContiguousFloat &getChassisFrameSetpoint() const
    {
        return chassisFrameSetpoint;
    }

    /// @return turret motor measurement relative to the chassis, in radians
    mockable inline const tap::algorithms::ContiguousFloat &getChassisFrameMeasuredAngle() const
    {
        return chassisFrameMeasuredAngle;
    }

    /// @return velocity of the turret, in rad/sec
    mockable inline float getChassisFrameVelocity() const { return (M_TWOPI / 60) * motor->getShaftRPM(); }

    mockable inline float getAngleFromCenter() const
    {
        return tap::algorithms::ContiguousFloat(
                   chassisFrameMeasuredAngle.getValue() - config.startAngle,
                   -M_PI,
                   M_PI)
            .getValue();
    }

    /// @return turret controller controlling this motor (as specified by `attachTurretController`)
    mockable const algorithms::TurretControllerInterface *getTurretController() const
    {
        return turretController;
    }

    mockable const TurretMotorConfig &getConfig() const { return config; }

private:
    const TurretMotorConfig config;

    /// Low-level motor object that this object interacts with
    tap::motor::MotorInterface *motor;

    /// Associated turret controller interface that is being used by a command to control this motor
    const algorithms::TurretControllerInterface *turretController;
    tap::algorithms::ContiguousFloat chassisFrameSetpoint;
    tap::algorithms::ContiguousFloat chassisFrameMeasuredAngle;

    uint16_t lastUpdatedEncoderValue;
};
}  // namespace aruwsrc::control::turret

#endif  // TURRET_MOTOR_HPP_
