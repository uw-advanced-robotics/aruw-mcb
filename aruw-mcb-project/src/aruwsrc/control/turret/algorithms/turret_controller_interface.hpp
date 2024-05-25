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

#ifndef TURRET_CONTROLLER_INTERFACE_HPP_
#define TURRET_CONTROLLER_INTERFACE_HPP_

#include "tap/algorithms/wrapped_float.hpp"

using namespace tap::algorithms;

namespace aruwsrc::control::turret
{
class TurretMotor;
}

namespace aruwsrc::control::turret::algorithms
{
/**
 * An interface describing the functionality of a turret controller. When implementing this class,
 * the user is responsible for designing a controller that will set the desired output of some
 * turret subsystem. Instances of this interface are designed to be used in a command. Using this
 * interface allows you to easily interchange which turret controller is being used for a particular
 * robot.
 *
 * @note All units of setpoints mentioned below are in radians, the same units that the
 * `TurretMotor` uses.
 */
class TurretControllerInterface
{
public:
    /**
     * @param[in] TurretMotor A `TurretMotor` object accessible for children objects to use.
     */
    TurretControllerInterface(TurretMotor &turretMotor) : turretMotor(turretMotor) {}

    /**
     * Initializes the controller, resetting any controllers and configuring any variables that need
     * to be set initially. Expected to be called once before the turret controller's
     * `runController` function is called.
     */
    virtual void initialize() = 0;

    /**
     * Main controller update loop. Expected that the controller is initialized and that this
     * function is only called when `isOnline` is `false`. Call periodically.
     *
     * @param[in] dt The time difference in milliseconds between previous and current call of
     * `runController`.
     * @param[in] desiredSetpoint The controller's desired setpoint in whatever frame the controller
     * is operating. Units radians.
     */
    virtual void runController(const uint32_t dt, const float desiredSetpoint) = 0;

    /**
     * Sets the controller setpoint, but doesn't run the controller.
     */
    virtual void setSetpoint(float desiredSetpoint) = 0;

    /**
     * @return The controller's setpoint, units radians. **Does not** have to be in the same
     * reference frame as the TurretSubsystem's `get<yaw|pitch>Setpoint` functions.
     */
    virtual WrappedFloat getSetpoint() const = 0;

    /**
     * @return The controller's measurement (current value of the system), units radians. **Does
     * not** have to be in the same reference frame as the TurretMotor's `getChassisFrame*`
     * functions. Does not need to be normalized.
     */
    virtual WrappedFloat getMeasurement() const = 0;

    /**
     * @return `false` if the turret controller should not be running, whether this is because the
     * turret is offline or some sensor the turret controller is using is invalid. Otherwise return
     * `true`.
     */
    virtual bool isOnline() const = 0;

    /**
     * Converts the passed in controllerFrameAngle from the controller frame to the chassis frame of
     * reference.
     *
     * @param[in] controllerFrameAngle Some angle (in radians) in the controller frame. Not required
     * to be normalized.
     * @return The controllerFrameAngle converted to the chassis frame, a value in radians that is
     * not required to be normalized.
     */
    virtual WrappedFloat convertControllerAngleToChassisFrame(
        WrappedFloat controllerFrameAngle) const = 0;

    /**
     * Converts the passed in controllerFrameAngle from the chassis frame to the controller frame of
     * reference.
     *
     * @param[in] chassisFrameAngle Some angle (in radians) in the chassis frame. Not required
     * to be normalized.
     * @return The chassisFrameAngle converted to the controller frame, a value in radians that is
     * not required to be normalized.
     */
    virtual WrappedFloat convertChassisAngleToControllerFrame(
        WrappedFloat chassisFrameAngle) const = 0;

protected:
    TurretMotor &turretMotor;
};

class TurretPitchControllerInterface : public TurretControllerInterface
{
public:
    TurretPitchControllerInterface(TurretMotor &turretMotor)
        : TurretControllerInterface(turretMotor)
    {
    }
};

class TurretYawControllerInterface : public TurretControllerInterface
{
public:
    TurretYawControllerInterface(TurretMotor &turretMotor) : TurretControllerInterface(turretMotor)
    {
    }
};
}  // namespace aruwsrc::control::turret::algorithms

#endif  // TURRET_CONTROLLER_INTERFACE_HPP_
