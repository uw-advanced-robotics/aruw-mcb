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

#ifndef TURRET_WORLD_RELATIVE_TURRET_IMU_COMMAND_HPP_
#define TURRET_WORLD_RELATIVE_TURRET_IMU_COMMAND_HPP_

#include "tap/algorithms/contiguous_float.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/command.hpp"

namespace tap
{
class Drivers;
}

namespace aruwsrc::chassis
{
class ChassisSubsystem;
}

namespace aruwsrc::control::turret
{
class TurretSubsystem;

/**
 * Turret control, with the pitch and yaw gimbals using the world relative frame, such that the
 * desired turret angle is independent of the direction that the chassis is facing
 * or rotating.
 *
 * @note Assumes that an IMU mounted on the turret frame is in communication with
 *      the MCU this code is running on via the `ImuRxListener`.
 */
class TurretWorldRelativeTurretImuCommand : public tap::control::Command
{
public:
    /**
     * This command requires the turret subsystem from a command/subsystem framework perspective.
     * The `ChassisSubsystem` is only used for for odometry information.
     */
    TurretWorldRelativeTurretImuCommand(
        tap::Drivers *drivers,
        TurretSubsystem *turretSubsystem,
        const chassis::ChassisSubsystem *chassisSubsystem);

    bool isReady() override;

    bool isFinished() const override;

    void initialize() override;

    void execute() override;

    void end(bool) override;

    const char *getName() const override { return "turret WR turret IMU"; }

private:
#ifdef TARGET_SOLDIER
    static constexpr float YAW_P = 4200.0f;
    static constexpr float YAW_I = 50.0f;
    static constexpr float YAW_D = 350.0f;
    static constexpr float YAW_MAX_ERROR_SUM = 1000.0f;
    static constexpr float YAW_MAX_OUTPUT = 30000.0f;
    static constexpr float YAW_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_R_DERIVATIVE_KALMAN = 10.0f;
    static constexpr float YAW_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float YAW_R_PROPORTIONAL_KALMAN = 10.0f;

    static constexpr float PITCH_P = 4700.0f;
    static constexpr float PITCH_I = 5.0f;
    static constexpr float PITCH_D = 180.0f;
    static constexpr float PITCH_MAX_ERROR_SUM = 3000.0f;
    static constexpr float PITCH_MAX_OUTPUT = 30000.0f;
    static constexpr float PITCH_Q_DERIVATIVE_KALMAN = 1.5f;
    static constexpr float PITCH_R_DERIVATIVE_KALMAN = 47.0f;
    static constexpr float PITCH_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float PITCH_R_PROPORTIONAL_KALMAN = 2.0f;

    static constexpr float USER_YAW_INPUT_SCALAR = 1.0f;
    static constexpr float USER_PITCH_INPUT_SCALAR = 1.0f;
#else
    static constexpr float YAW_P = 0.0f;
    static constexpr float YAW_I = 0.0f;
    static constexpr float YAW_D = 0.0f;
    static constexpr float YAW_MAX_ERROR_SUM = 0.0f;
    static constexpr float YAW_MAX_OUTPUT = 0.0f;
    static constexpr float YAW_Q_DERIVATIVE_KALMAN = 0.0f;
    static constexpr float YAW_R_DERIVATIVE_KALMAN = 0.0f;
    static constexpr float YAW_Q_PROPORTIONAL_KALMAN = 0.0f;
    static constexpr float YAW_R_PROPORTIONAL_KALMAN = 0.0f;

    static constexpr float PITCH_P = 0.0f;
    static constexpr float PITCH_I = 0.0f;
    static constexpr float PITCH_D = 0.0f;
    static constexpr float PITCH_MAX_ERROR_SUM = 0.0f;
    static constexpr float PITCH_MAX_OUTPUT = 0.0f;
    static constexpr float PITCH_Q_DERIVATIVE_KALMAN = 0.0f;
    static constexpr float PITCH_R_DERIVATIVE_KALMAN = 0.0f;
    static constexpr float PITCH_Q_PROPORTIONAL_KALMAN = 0.0f;
    static constexpr float PITCH_R_PROPORTIONAL_KALMAN = 0.0f;

    static constexpr float USER_YAW_INPUT_SCALAR = 0.0f;
    static constexpr float USER_PITCH_INPUT_SCALAR = 0.0f;
#endif

    tap::Drivers *drivers;

    TurretSubsystem *turretSubsystem;
    const chassis::ChassisSubsystem *chassisSubsystem;

    tap::algorithms::ContiguousFloat worldFrameYawSetpoint;
    tap::algorithms::ContiguousFloat worldFramePitchSetpoint;

    uint32_t prevTime;

    // Pitch/yaw PID controllers
    tap::algorithms::SmoothPid yawPid;
    tap::algorithms::SmoothPid pitchPid;

    int imuMessageReceivedLEDBlinkCounter = 0;

    void runYawPositionController(uint32_t dt);
    void runPitchPositionController(uint32_t dt);
};  // class TurretWorldRelativeTurretImuCommand

}  // namespace aruwsrc::control::turret

#endif  // TURRET_WORLD_RELATIVE_TURRET_IMU_COMMAND_HPP_
