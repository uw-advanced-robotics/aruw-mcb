/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef SENTRY_TURRET_MAJOR_WORLD_RELATIVE_YAW_CONTROLLER_HPP_
#define SENTRY_TURRET_MAJOR_WORLD_RELATIVE_YAW_CONTROLLER_HPP_

#include <cstdint>

#include "tap/algorithms/fuzzy_pd.hpp"
#include "tap/algorithms/wrapped_float.hpp"

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/control/turret/algorithms/turret_controller_interface.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"

// @todo primarily here to avoid namespace pain
#include "tap/algorithms/transforms/transform.hpp"

#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_transforms.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_minor_subsystem.hpp"

namespace aruwsrc::control::turret
{
class TurretMotor;
}

namespace aruwsrc::control::turret::algorithms
{
/**
 * World frame turret major yaw controller for the sentry.
 *
 * Runs a cascade PID controller (position PID output feeds into velocity PID controller, velocity
 * PID controller is desired motor output) to control the turret yaw.
 *
 * @note Includes turret minor torque compensation and uses chassis angular velocity. Has
 * feedforward.
 *
 * Implements TurretControllerInterface interface, see parent class comment for details.
 */
class TurretMajorWorldFrameController final : public TurretYawControllerInterface
{
public:
    /**
     * @param[in] worldToChassis An self-updating reference to a transform from the world frame
     *          to the chassis frame, used to determine the pose of the chassis relative to the
     *          world frame.
     * @param[in] chassis A chassis subsystem for getting angular velocity of the chassis.
     * @param[in] yawMotor A `TurretMotor` object accessible for children objects to use.
     * @param[in] turretLeft The left turret minor.
     * @param[in] turretRight The right turret minor.
     * @param[in] positionPid Position PID controller.
     * @param[in] velocityPid Velocity PID controller.
     * @param[in] maxVelErrorInput Cap on the max error passed into velocity controller.
     * @param[in] minorMajorTorqueRatio Gain on the torque compensation.
     * @param[in] feedforwardGain Gain on the feedforward term of the final output.
     */
    TurretMajorWorldFrameController(
        const tap::algorithms::transforms::Transform& worldToChassis,
        const aruwsrc::chassis::HolonomicChassisSubsystem& chassis,
        TurretMotor& yawMotor,
        const aruwsrc::control::sentry::SentryTurretMinorSubsystem& turretLeft,
        const aruwsrc::control::sentry::SentryTurretMinorSubsystem& turretRight,
        tap::algorithms::SmoothPid& positionPid,
        tap::algorithms::SmoothPid& velocityPid,
        float maxVelErrorInput,
        float minorMajorTorqueRatio,
        float feedforwardGain);

    void initialize() final;

    /**
     * @see TurretControllerInterface for more details.
     * @param[in] desiredSetpoint The yaw desired setpoint in the world frame.
     */
    void runController(const uint32_t dt, const float desiredSetpoint) final;

    /// Sets the world frame yaw angle setpoint, refer to top level documentation for more details.
    void setSetpoint(float desiredSetpoint) final;

    /// @return World frame yaw angle setpoint, refer to top level documentation for more details.
    float getSetpoint() const final;

    /// @return World frame yaw angle measurement, refer to top level documentation for more
    /// details.
    float getMeasurement() const final;

    bool isOnline() const final;

    // @todo see todo in interface class
    float convertControllerAngleToChassisFrame(float controllerFrameAngle) const final
    {
        controllerFrameAngle = controllerFrameAngle;  // to make pipeline not complain
        return 0.0;
    };

    float convertChassisAngleToControllerFrame(float chassisFrameAngle) const final
    {
        chassisFrameAngle = chassisFrameAngle;  // to make pipelien not complain
        return 0.0;
    };

private:
    const tap::algorithms::transforms::Transform& worldToChassis;

    const aruwsrc::chassis::HolonomicChassisSubsystem& chassis;

    TurretMotor& yawMotor;

    const aruwsrc::control::sentry::SentryTurretMinorSubsystem& turretLeft;
    const aruwsrc::control::sentry::SentryTurretMinorSubsystem& turretRight;

    tap::algorithms::SmoothPid& positionPid;
    tap::algorithms::SmoothPid& velocityPid;

    tap::algorithms::WrappedFloat worldFrameSetpoint;
    float positionPidOutput;
    float torqueCompensation = 0.0f;

    float maxVelErrorInput;

    float minorMajorTorqueRatio;

    float feedforwardGain;
};

}  // namespace aruwsrc::control::turret::algorithms

#endif  // SENTRY_TURRET_MAJOR_WORLD_RELATIVE_YAW_CONTROLLER_HPP_
