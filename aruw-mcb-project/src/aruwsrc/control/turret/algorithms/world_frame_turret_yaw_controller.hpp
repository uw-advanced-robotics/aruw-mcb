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

#ifndef WORLD_FRAME_TURRET_IMU_TURRET_CONTROLLER_HPP_
#define WORLD_FRAME_TURRET_IMU_TURRET_CONTROLLER_HPP_

#include <cstdint>

#include "tap/algorithms/contiguous_float.hpp"
#include "tap/algorithms/fuzzy_pd.hpp"

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"

#include "turret_controller_interface.hpp"
#include "tap/algorithms/contiguous_float.hpp"

// @todo primarily here to avoid namespace pain
#include "aruwsrc/robot/sentry/sentry_transforms.hpp"
#include "tap/algorithms/transforms/transform.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_minor_subsystem.hpp"

namespace aruwsrc::control::turret
{
class TurretMotor;
}

namespace aruwsrc::control::turret::algorithms
{
/**
 * World frame turret yaw controller. Requires that a development board be mounted rigidly on the
 * turret and connected via the `TurretMCBCanComm` class. The development board's IMU is used to
 * determine the turret's world frame coordinates directly, making this controller better than the
 * `WorldFrameChassisImuTurretController`.
 *
 * Runs a cascade PID controller (position PID output feeds into velocity PID controller, velocity
 * PID controller is desired motor output) to control the turret yaw.
 *
 * Implements TurretControllerInterface interface, see parent class comment for details.
 */
class WorldFrameTurretYawCascadePIDController final : public TurretYawControllerInterface
{
public:
    /**
     * @param[in] turretMCBCanComm A TurretMCBCanComm object that will be queried for IMU
     * information.
     * @param[in] yawMotor A `TurretMotor` object accessible for children objects to use.
     * @param[in] positionPid Position PID controller.
     * @param[in] velocityPid Velocity PID controller.
     */
    WorldFrameTurretYawCascadePIDController(
        const tap::algorithms::transforms::Transform<aruwsrc::sentry::WorldFrame, aruwsrc::sentry::ChassisFrame>& worldToBaseTransform,
        const aruwsrc::chassis::HolonomicChassisSubsystem& chassis,
        TurretMotor &yawMotor,
        const aruwsrc::control::turret::SentryTurretMinorSubsystem& girlboss,
        const aruwsrc::control::turret::SentryTurretMinorSubsystem& malewife,
        tap::algorithms::SmoothPid &positionPid,
        tap::algorithms::SmoothPid &velocityPid,
        float maxVelErrorInput,
        float minorMajorTorqueRatio);

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
    float convertControllerAngleToChassisFrame(float controllerFrameAngle) const final { return 0.0; };

    float convertChassisAngleToControllerFrame(float chassisFrameAngle) const final { return 0.0; };

// void overWrite

private:
    const tap::algorithms::transforms::Transform<aruwsrc::sentry::WorldFrame, aruwsrc::sentry::ChassisFrame>& worldToBaseTransform;

    const aruwsrc::chassis::HolonomicChassisSubsystem& chassis;

    const aruwsrc::control::turret::SentryTurretMinorSubsystem& girlboss;
    const aruwsrc::control::turret::SentryTurretMinorSubsystem& malewife;

    tap::algorithms::SmoothPid &positionPid;
    tap::algorithms::SmoothPid &velocityPid;

    tap::algorithms::ContiguousFloat worldFrameSetpoint;

    float positionPidOutput;
    float torqueCompensation = 0.0f;

    TurretMotor& yawMotor;

    float maxVelErrorInput;

    float minorMajorTorqueRatio;
};

}  // namespace aruwsrc::control::turret::algorithms

#endif  //  WORLD_FRAME_TURRET_IMU_TURRET_CONTROLLER_HPP_
