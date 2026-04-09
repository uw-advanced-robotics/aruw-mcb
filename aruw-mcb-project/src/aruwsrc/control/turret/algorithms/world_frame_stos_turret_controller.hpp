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

#ifndef WORLD_FRAME_STOS_TURRET_CONTROLLER_HPP_
#define WORLD_FRAME_STOS_TURRET_CONTROLLER_HPP_

#include <cstdint>

#include "tap/algorithms/filter/butterworth.hpp"
#include "tap/algorithms/filter/discrete_filter.hpp"
#include "tap/algorithms/fuzzy_pd.hpp"
#include "tap/algorithms/transforms/transform.hpp"
#include "tap/algorithms/wrapped_float.hpp"

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"

#include "turret_controller_interface.hpp"
#include "turret_setpoint_kalman.hpp"

using namespace tap::algorithms;

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
 * Runs a STOS optimal controller with a LQR in the small error region.
 *
 * Implements TurretControllerInterface interface, see parent class comment for details.
 */
template <Axis AXIS>
class WorldFrameTurretImuSTOSTurretController final : public TurretAxisControllerInterface<AXIS>
{
public:
    /**
     * @param[in] worldToTurret A Transform object that will be queried for orientation
     * information.
     * @param[in] motor A `TurretMotor` object accessible for children objects to use.
     * @param[in] positionPid Position PID controller.
     * @param[in] velocityPid Velocity PID controller.
     */
    WorldFrameTurretImuSTOSTurretController(
        const transforms::Transform &worldToTurret,
        const aruwsrc::communication::can::TurretMCBCanComm &turretMCBCanComm,
        TurretMotor &turretMotor,
        OptimalSTOSController::STOSConstants constants,
        float lqrT,
        float lqrTd,
        const std::vector<TurretCompensatorInterface *> compensators = {});

    void initialize() final;

    /**
     * @see TurretControllerInterface for more details.
     * @param[in] desiredSetpoint The pitch desired setpoint in the world frame.
     */
    void runController(const float dt, const WrappedFloat desiredSetpoint) final;

    /// Sets the world frame pitch angle setpoint, refer to top level documentation for more
    /// details.
    void setSetpoint(WrappedFloat desiredSetpoint) final;

    /// @return World frame pitch angle setpoint, refer to top level documentation for more
    // details.
    inline WrappedFloat getSetpoint() const final { return worldFrameSetpoint; }

    /// @return World frame pitch angle setpoint, refer to top level documentation for more
    // details.
    WrappedFloat getMeasurement() const final;

    bool isOnline() const final;

    WrappedFloat convertControllerAngleToChassisFrame(
        WrappedFloat controllerFrameAngle) const final;

    WrappedFloat convertChassisAngleToControllerFrame(WrappedFloat chassisFrameAngle) const final;

private:
    const transforms::Transform &worldToTurret;
    const aruwsrc::communication::can::TurretMCBCanComm &turretMCBCanComm;

    OptimalSTOSController stosController;
    float lqrT;
    float lqrTd;

    WrappedFloat worldFrameSetpoint;
    TurretSetpointKalmanFilter setpointFilter;

    float DEBUG1{0.08f};
    float DEBUG2{0.0f};
    float DEBUG3{0.0f};
    float DEBUGV{0.0f};
    float DEBUGP{0.0f};
    float B_DAMP{0.0443f};
    float J_FF{0.0073f};
    float FRICTION_FF{0.001f};
};
}  // namespace aruwsrc::control::turret::algorithms

#endif  //  WORLD_FRAME_STOS_TURRET_CONTROLLER_HPP_
#include "world_frame_stos_turret_controller_impl.hpp"