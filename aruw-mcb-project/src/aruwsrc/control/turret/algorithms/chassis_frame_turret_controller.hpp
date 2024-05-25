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

#ifndef CHASSIS_FRAME_TURRET_CONTROLLER_HPP_
#define CHASSIS_FRAME_TURRET_CONTROLLER_HPP_

#include <cstdint>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/algorithms/wrapped_float.hpp"

#include "turret_controller_interface.hpp"

using namespace tap::algorithms;

namespace aruwsrc::control::turret
{
class TurretMotor;
}

namespace aruwsrc::control::turret::algorithms
{
/**
 * Controller that runs a single position PID controller in the chassis frame to control the turret
 * yaw.
 *
 * Implements TurretControllerInterface interface, see parent class comment for details.
 */
class ChassisFrameYawTurretController final : public TurretYawControllerInterface
{
public:
    /**
     * @param[in] yawMotor A `TurretMotor` object accessible for children objects to use.
     * @param[in] pidConfig PID configuration struct for the controller.
     */
    ChassisFrameYawTurretController(
        TurretMotor &yawMotor,
        const tap::algorithms::SmoothPidConfig &pidConfig);

    void initialize() final;

    /**
     * @see TurretControllerInterface for more details.
     * @param[in] desiredSetpoint The yaw desired setpoint in the chassis frame.
     */
    void runController(const uint32_t dt, const float desiredSetpoint) final;

    void setSetpoint(float desiredSetpoint) final;

    /// @return The chassis frame yaw turret measurement, refer to top level documentation for more
    /// details.
    WrappedFloat getMeasurement() const final;

    /**
     * @return The yaw setpoint, in the chassis frame.
     */
    WrappedFloat getSetpoint() const final;

    bool isOnline() const final;

    /// Since the controller is in the chassis frame, no frame transformation is required.
    inline WrappedFloat convertControllerAngleToChassisFrame(
        WrappedFloat controllerFrameAngle) const final
    {
        return controllerFrameAngle;
    }

    /// Since the controller is in the chassis frame, no frame transformation is required.
    inline WrappedFloat convertChassisAngleToControllerFrame(
        WrappedFloat chassisFrameAngle) const final
    {
        return chassisFrameAngle;
    }

private:
    tap::algorithms::SmoothPid pid;
};

/**
 * Controller that runs a single position PID controller in the chassis frame to control the turret
 * pitch.
 *
 * Implements TurretControllerInterface interface, see parent class comment for details.
 */
class ChassisFramePitchTurretController final : public TurretPitchControllerInterface
{
public:
    /**
     * @param[in] pitchMotor A `TurretMotor` object accessible for children objects to use.
     * @param[in] pidConfig PID configuration struct for the controller.
     */
    ChassisFramePitchTurretController(
        TurretMotor &pitchMotor,
        const tap::algorithms::SmoothPidConfig &pidConfig);

    void initialize() final;

    /**
     * @see TurretControllerInterface for more details.
     * @param[in] desiredSetpoint The pitch desired setpoint in the chassis frame.
     */
    void runController(const uint32_t dt, const float desiredSetpoint) final;

    void setSetpoint(float desiredSetpoint) final;

    /**
     * @return The pitch setpoint, in the chassis frame.
     */
    WrappedFloat getSetpoint() const final;

    /// @return The chassis frame pitch turret measurement, refer to top level documentation for
    /// more details.
    WrappedFloat getMeasurement() const final;

    bool isOnline() const final;

    /// Since the controller is in the chassis frame, no frame transformation is required.
    inline WrappedFloat convertControllerAngleToChassisFrame(
        WrappedFloat controllerFrameAngle) const final
    {
        return controllerFrameAngle;
    }

    /// Since the controller is in the chassis frame, no frame transformation is required.
    inline WrappedFloat convertChassisAngleToControllerFrame(
        WrappedFloat chassisFrameAngle) const final
    {
        return chassisFrameAngle;
    }

private:
    tap::algorithms::SmoothPid pid;
};

}  // namespace aruwsrc::control::turret::algorithms

#endif  // CHASSIS_FRAME_TURRET_CONTROLLER_HPP_
