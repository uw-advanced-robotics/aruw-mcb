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

#ifndef TURRET_WORLD_RELATIVE_CHASSIS_IMU_COMMAND_HPP_
#define TURRET_WORLD_RELATIVE_CHASSIS_IMU_COMMAND_HPP_

#include "tap/algorithms/contiguous_float.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/command.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc
{
namespace chassis
{
class ChassisSubsystem;
}

namespace control::turret
{
class TurretSubsystem;

/**
 * Turret control, with the yaw gimbal using the world relative frame, such that the
 * desired turret angle is independent of the direction that the chassis is facing
 * or rotating. Assumes the board running this subsystem is a RoboMaster type A
 * board with an Mpu6500 and that this board is mounted statically on the chassis.
 *
 * This command is a vessel that calls functions defined in
 * `world_frame_chassis_imu_turret_controller.hpp` using the user input as the turret setpoint. This
 * command contains the necessary variables to call the functions
 * `WorldFrameChassisImuTurretController::runCascadePitchPidController` and
 * `WorldFrameChassisImuTurretController::runCascadePidController`. This command will then just call
 * these  functions, which will run the turret controller and command the turret subsystem to move
 * to the desired setpoint.
 *
 * @note This command is being phased out by the TurretWorldRelativeTurretIMUCommand
 *      but is still maintained since many turrets don't have IMUs on them. Also its
 *      good to have in case the turret IMU is broken or disconnected.
 */
class TurretWorldRelativeChassisImuCommand : public tap::control::Command
{
public:
    /**
     * This command requires the turret subsystem from a command/subsystem framework perspective.
     */
    TurretWorldRelativeChassisImuCommand(
        aruwsrc::Drivers *drivers,
        TurretSubsystem *turretSubsystem);

    void initialize() override;

    bool isReady() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool) override;

    const char *getName() const override { return "turret WR chassis IMU"; }

private:
#if ALL_SOLDIERS
    static constexpr float YAW_P = 3500.0f;
    static constexpr float YAW_I = 0.0f;
    static constexpr float YAW_D = 190.0f;
    static constexpr float YAW_MAX_ERROR_SUM = 0.0f;
    static constexpr float YAW_MAX_OUTPUT = 32000.0f;
    static constexpr float YAW_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_R_DERIVATIVE_KALMAN = 40.0f;
    static constexpr float YAW_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float YAW_R_PROPORTIONAL_KALMAN = 0.0f;

    static constexpr float PITCH_P = 4000.0f;
    static constexpr float PITCH_I = 0.0f;
    static constexpr float PITCH_D = 130.0f;
    static constexpr float PITCH_MAX_ERROR_SUM = 0.0f;
    static constexpr float PITCH_MAX_OUTPUT = 32000.0f;
    static constexpr float PITCH_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float PITCH_R_DERIVATIVE_KALMAN = 10.0f;
    static constexpr float PITCH_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float PITCH_R_PROPORTIONAL_KALMAN = 2.0f;
#else
    static constexpr float YAW_P = 2200.0f;
    static constexpr float YAW_I = 50.0f;
    static constexpr float YAW_D = 60.0f;
    static constexpr float YAW_MAX_ERROR_SUM = 1000.0f;
    static constexpr float YAW_MAX_OUTPUT = 30000.0f;
    static constexpr float YAW_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_R_DERIVATIVE_KALMAN = 50.0f;
    static constexpr float YAW_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float YAW_R_PROPORTIONAL_KALMAN = 10.0f;

    static constexpr float PITCH_P = 3400.0f;
    static constexpr float PITCH_I = 0.0f;
    static constexpr float PITCH_D = 150.0f;
    static constexpr float PITCH_MAX_ERROR_SUM = 0.0f;
    static constexpr float PITCH_MAX_OUTPUT = 30000.0f;
    static constexpr float PITCH_Q_DERIVATIVE_KALMAN = 1.5f;
    static constexpr float PITCH_R_DERIVATIVE_KALMAN = 47.0f;
    static constexpr float PITCH_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float PITCH_R_PROPORTIONAL_KALMAN = 2.0f;
#endif

    /**
     * Scales how much user input from `ControlOperatorInterface` change the setpoint
     * of this command. Basically: mouse sensitivity
     */
    static constexpr float USER_YAW_INPUT_SCALAR = 1.0f;
    static constexpr float USER_PITCH_INPUT_SCALAR = 1.0f;

    aruwsrc::Drivers *drivers;

    TurretSubsystem *turretSubsystem;

    // Yaw related values
    tap::algorithms::ContiguousFloat worldFrameYawSetpoint;
    float chassisFrameInitImuYawAngle;

    uint32_t prevTime;

    // Pitch/yaw PID controllers
    tap::algorithms::SmoothPid yawPid;
    tap::algorithms::SmoothPid pitchPid;
};  // class TurretWorldRelativeChassisImuCommand

}  // namespace control::turret

}  // namespace aruwsrc

#endif  // TURRET_WORLD_RELATIVE_CHASSIS_IMU_COMMAND_HPP_
