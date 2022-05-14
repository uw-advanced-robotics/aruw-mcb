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
class WorldFrameYawTurretImuCascadePidTurretController final : public TurretYawControllerInterface
{
public:
    /**
     * @param[in] drivers A drivers object that will be queried for IMU information.
     * @param[in] yawMotor A `TurretMotor` object accessible for children objects to use.
     * @param[in] posPidConfig Position PID configuration struct for the controller.
     * @param[in] velPidConfig Velocity PID configuration struct for the controller.
     */
    WorldFrameYawTurretImuCascadePidTurretController(
        const aruwsrc::can::TurretMCBCanComm &turretMCBCanComm,
        TurretMotor *yawMotor,
        const tap::algorithms::SmoothPidConfig &posPidConfig,
        const tap::algorithms::SmoothPidConfig &velPidConfig);

    void initialize() final;

    /**
     * @see TurretControllerInterface for more details.
     * @param[in] desiredSetpoint The yaw desired setpoint in the world frame.
     */
    void runController(const uint32_t dt, const float desiredSetpoint) final;

    void setSetpoint(float desiredSetpoint) final;

    float getSetpoint() const final;

    bool isOnline() const final;

private:
    const aruwsrc::can::TurretMCBCanComm &turretMCBCanComm;

    tap::algorithms::SmoothPid positionPid;
    tap::algorithms::SmoothPid velocityPid;

    tap::algorithms::ContiguousFloat worldFrameSetpoint;
};

/**
 * Band-aid fix since I don't want to fix WorldFrameYawTurretImuCascadePidTurretController's
 * constructor.
 */
class HeroTurretImuCascadePidTurretController final : public TurretYawControllerInterface
{
public:
    HeroTurretImuCascadePidTurretController(
        const aruwsrc::can::TurretMCBCanComm &turretMCBCanComm,
        TurretMotor *yawMotor,
        const tap::algorithms::SmoothPidConfig &posPidConfig,
        const tap::algorithms::FuzzyPDConfig &fuzzyPidConfig,
        const tap::algorithms::SmoothPidConfig &velPidConfig);

    void initialize() final;

    /**
     * @see TurretControllerInterface for more details.
     * @param[in] desiredSetpoint The yaw desired setpoint in the world frame.
     */
    void runController(const uint32_t dt, const float desiredSetpoint) final;

    void setSetpoint(float desiredSetpoint) final;

    float getSetpoint() const final;

    bool isOnline() const final;

private:
    const aruwsrc::can::TurretMCBCanComm &turretMCBCanComm;

    tap::algorithms::FuzzyPD positionPid;
    tap::algorithms::SmoothPid velocityPid;

    tap::algorithms::ContiguousFloat worldFrameSetpoint;
};

/**
 * World frame turret pitch controller. Requires that a development board be mounted rigidly on the
 * turret and connected via the `TurretMCBCanComm` class. The development board's IMU is used to
 * determine the turret's world frame coordinates directly, making this controller better than the
 * `WorldFrameChassisImuTurretController`.
 *
 * Runs a cascade PID controller (position PID output feeds into velocity PID controller, velocity
 * PID controller is desired motor output) to control the turret pitch.
 *
 * Implements TurretControllerInterface interface, see parent class comment for details.
 */
class WorldFramePitchTurretImuCascadePidTurretController final
    : public TurretPitchControllerInterface
{
public:
    /**
     * @param[in] drivers A drivers object that will be queried for IMU information.
     * @param[in] pitchMotor A `TurretMotor` object accessible for children objects to use.
     * @param[in] posPidConfig Position PID configuration struct for the controller.
     * @param[in] velPidConfig Velocity PID configuration struct for the controller.
     */
    WorldFramePitchTurretImuCascadePidTurretController(
        const aruwsrc::can::TurretMCBCanComm &turretMCBCanComm,
        TurretMotor *pitchMotor,
        const tap::algorithms::SmoothPidConfig &posPidConfig,
        const tap::algorithms::SmoothPidConfig &velPidConfig);

    void initialize() final;

    /**
     * @see TurretControllerInterface for more details.
     * @param[in] desiredSetpoint The pitch desired setpoint in the world frame.
     */
    void runController(const uint32_t dt, const float desiredSetpoint) final;

    void setSetpoint(float desiredSetpoint) final;

    float getSetpoint() const final;

    bool isOnline() const final;

private:
    const aruwsrc::can::TurretMCBCanComm &turretMCBCanComm;

    tap::algorithms::SmoothPid positionPid;
    tap::algorithms::SmoothPid velocityPid;

    tap::algorithms::ContiguousFloat worldFrameSetpoint;
};
}  // namespace aruwsrc::control::turret::algorithms

#endif  //  WORLD_FRAME_TURRET_IMU_TURRET_CONTROLLER_HPP_
