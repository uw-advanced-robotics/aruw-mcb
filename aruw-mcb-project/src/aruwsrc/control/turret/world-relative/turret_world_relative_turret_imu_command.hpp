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

namespace aruwsrc
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
 * This command is a vessel that calls functions defined in
 * `world_frame_turret_imu_turret_controller.hpp` using the user input as the turret setpoint. This
 * command contains the necessary variables to call the functions
 * `WorldFrameTurretImuTurretController::runCascadePitchPidController` and
 * `WorldFrameTurretImuTurretController::runCascadePidController`. This command will then just call
 * these  functions, which will run the turret controller and command the turret subsystem to move
 * to the desired setpoint.
 *
 * @note Assumes that an IMU mounted on the turret is in communication with
 *      the MCU this code is running on via the `TurretMCBCanComm`.
 */
class TurretWorldRelativeTurretImuCommand : public tap::control::Command
{
public:
    /**
     * This command requires the turret subsystem from a command/subsystem framework perspective.
     */
    TurretWorldRelativeTurretImuCommand(
        aruwsrc::Drivers *drivers,
        TurretSubsystem *turretSubsystem);

    bool isReady() override;

    void initialize() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool) override;

    const char *getName() const override { return "turret WR turret IMU"; }

private:
    static constexpr float YAW_POS_P = 12.0f;
    static constexpr float YAW_POS_I = 0.0f;
    static constexpr float YAW_POS_D = 0.0f;
    static constexpr float YAW_POS_MAX_ERROR_SUM = 60.0f;
    static constexpr float YAW_POS_MAX_OUTPUT = 7000.0f;
    static constexpr float YAW_POS_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_POS_R_DERIVATIVE_KALMAN = 0.0f;
    static constexpr float YAW_POS_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float YAW_POS_R_PROPORTIONAL_KALMAN = 0.0f;
    static constexpr float YAW_POS_DEADZONE = 0.0f;

    static constexpr float YAW_VEL_P = 280.0f;
    static constexpr float YAW_VEL_I = 10.0f;
    static constexpr float YAW_VEL_D = 0.0f;
    static constexpr float YAW_VEL_MAX_ERROR_SUM = 10000.0f;
    static constexpr float YAW_VEL_MAX_OUTPUT = 30000.0f;
    static constexpr float YAW_VEL_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_VEL_R_DERIVATIVE_KALMAN = 0.0f;
    static constexpr float YAW_VEL_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float YAW_VEL_R_PROPORTIONAL_KALMAN = 0.5f;
    static constexpr float YAW_VEL_DEADZONE = 0.0f;

    static constexpr float PITCH_POS_P = 15.0f;
    static constexpr float PITCH_POS_I = 0.0f;
    static constexpr float PITCH_POS_D = 0.0f;
    static constexpr float PITCH_POS_MAX_ERROR_SUM = 80.0f;
    static constexpr float PITCH_POS_MAX_OUTPUT = 7000.0f;
    static constexpr float PITCH_POS_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float PITCH_POS_R_DERIVATIVE_KALMAN = 0.0f;
    static constexpr float PITCH_POS_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float PITCH_POS_R_PROPORTIONAL_KALMAN = 0.0f;
    static constexpr float PITCH_POS_DEADZONE = 0.0f;

    static constexpr float PITCH_VEL_P = 280.0f;
    static constexpr float PITCH_VEL_I = 8.0f;
    static constexpr float PITCH_VEL_D = 0.0f;
    static constexpr float PITCH_VEL_MAX_ERROR_SUM = 8000.0f;
    static constexpr float PITCH_VEL_MAX_OUTPUT = 30000.0f;
    static constexpr float PITCH_VEL_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float PITCH_VEL_R_DERIVATIVE_KALMAN = 0.0f;
    static constexpr float PITCH_VEL_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float PITCH_VEL_R_PROPORTIONAL_KALMAN = 0.5f;
    static constexpr float PITCH_VEL_DEADZONE = 0.0f;

    /**
     * Scales how much user input from `ControlOperatorInterface` change the setpoint
     * of this command. Basically: mouse sensitivity
     */
    static constexpr float USER_YAW_INPUT_SCALAR = 1.0f;
    static constexpr float USER_PITCH_INPUT_SCALAR = 1.0f;

    aruwsrc::Drivers *drivers;

    TurretSubsystem *turretSubsystem;

    tap::algorithms::ContiguousFloat worldFrameYawSetpoint;
    tap::algorithms::ContiguousFloat worldFramePitchSetpoint;

    uint32_t prevTime;

    // Pitch/yaw PID controllers
    tap::algorithms::SmoothPid yawPosPid;
    tap::algorithms::SmoothPid yawVelPid;
    tap::algorithms::SmoothPid pitchPosPid;
    tap::algorithms::SmoothPid pitchVelPid;
};  // class TurretWorldRelativeTurretImuCommand

}  // namespace aruwsrc::control::turret

#endif  // TURRET_WORLD_RELATIVE_TURRET_IMU_COMMAND_HPP_
