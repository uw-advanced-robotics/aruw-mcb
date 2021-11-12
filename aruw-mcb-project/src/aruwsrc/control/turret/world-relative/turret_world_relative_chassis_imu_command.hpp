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

namespace tap
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
    TurretWorldRelativeChassisImuCommand(tap::Drivers *drivers, TurretSubsystem *turretSubsystem);

    void initialize() override;

    bool isReady() override;

    bool isFinished() const override;

    void execute() override;

    void end(bool) override;

    const char *getName() const override { return "turret WR chassis IMU"; }

private:
#if defined(TARGET_SOLDIER)
    static constexpr float YAW_POS_P = 15.0f;
    static constexpr float YAW_POS_I = 0.0f;
    static constexpr float YAW_POS_D = 0.0f;
    static constexpr float YAW_POS_MAX_ERROR_SUM = 60.0f;
    static constexpr float YAW_POS_MAX_OUTPUT = 7000.0f;
    static constexpr float YAW_POS_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_POS_R_DERIVATIVE_KALMAN = 0.0f;
    static constexpr float YAW_POS_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float YAW_POS_R_PROPORTIONAL_KALMAN = 0.0f;
    static constexpr float YAW_POS_DEADZONE = 0.0f;

    static constexpr float YAW_VEL_P = 110.0f;
    static constexpr float YAW_VEL_I = 4.0f;
    static constexpr float YAW_VEL_D = 0.0f;
    static constexpr float YAW_VEL_MAX_ERROR_SUM = 4000.0f;
    static constexpr float YAW_VEL_MAX_OUTPUT = 30000.0f;
    static constexpr float YAW_VEL_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_VEL_R_DERIVATIVE_KALMAN = 0.0f;
    static constexpr float YAW_VEL_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float YAW_VEL_R_PROPORTIONAL_KALMAN = 1.7f;
    static constexpr float YAW_VEL_DEADZONE = 1.0f;

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

    static constexpr float PITCH_VEL_P = 200.0f;
    static constexpr float PITCH_VEL_I = 0.0f;
    static constexpr float PITCH_VEL_D = 0.0f;
    static constexpr float PITCH_VEL_MAX_ERROR_SUM = 10000.0f;
    static constexpr float PITCH_VEL_MAX_OUTPUT = 30000.0f;
    static constexpr float PITCH_VEL_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float PITCH_VEL_R_DERIVATIVE_KALMAN = 0.0f;
    static constexpr float PITCH_VEL_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float PITCH_VEL_R_PROPORTIONAL_KALMAN = 13.0f;
    static constexpr float PITCH_VEL_DEADZONE = 1.0f;
#elif defined(TARGET_OLD_SOLDIER)
    static constexpr float YAW_POS_P = 20.0f;
    static constexpr float YAW_POS_I = 0.0f;
    static constexpr float YAW_POS_D = 0.0f;
    static constexpr float YAW_POS_MAX_ERROR_SUM = 60.0f;
    static constexpr float YAW_POS_MAX_OUTPUT = 7000.0f;
    static constexpr float YAW_POS_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_POS_R_DERIVATIVE_KALMAN = 0.0f;
    static constexpr float YAW_POS_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float YAW_POS_R_PROPORTIONAL_KALMAN = 0.0f;
    static constexpr float YAW_POS_DEADZONE = 0.0f;

    static constexpr float YAW_VEL_P = 130.0f;
    static constexpr float YAW_VEL_I = 4.0f;
    static constexpr float YAW_VEL_D = 0.0f;
    static constexpr float YAW_VEL_MAX_ERROR_SUM = 4000.0f;
    static constexpr float YAW_VEL_MAX_OUTPUT = 30000.0f;
    static constexpr float YAW_VEL_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_VEL_R_DERIVATIVE_KALMAN = 0.0f;
    static constexpr float YAW_VEL_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float YAW_VEL_R_PROPORTIONAL_KALMAN = 4.0f;
    static constexpr float YAW_VEL_DEADZONE = 1.0f;

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

    static constexpr float PITCH_VEL_P = 100.0f;
    static constexpr float PITCH_VEL_I = 1.0f;
    static constexpr float PITCH_VEL_D = 0.0f;
    static constexpr float PITCH_VEL_MAX_ERROR_SUM = 2000.0f;
    static constexpr float PITCH_VEL_MAX_OUTPUT = 30000.0f;
    static constexpr float PITCH_VEL_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float PITCH_VEL_R_DERIVATIVE_KALMAN = 0.0f;
    static constexpr float PITCH_VEL_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float PITCH_VEL_R_PROPORTIONAL_KALMAN = 5.0f;
    static constexpr float PITCH_VEL_DEADZONE = 1.0f;
#else
    static constexpr float YAW_POS_P = 15.0f;
    static constexpr float YAW_POS_I = 0.0f;
    static constexpr float YAW_POS_D = 0.0f;
    static constexpr float YAW_POS_MAX_ERROR_SUM = 60.0f;
    static constexpr float YAW_POS_MAX_OUTPUT = 7000.0f;
    static constexpr float YAW_POS_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_POS_R_DERIVATIVE_KALMAN = 0.0f;
    static constexpr float YAW_POS_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float YAW_POS_R_PROPORTIONAL_KALMAN = 0.0f;
    static constexpr float YAW_POS_DEADZONE = 0.0f;

    static constexpr float YAW_VEL_P = 110.0f;
    static constexpr float YAW_VEL_I = 4.0f;
    static constexpr float YAW_VEL_D = 0.0f;
    static constexpr float YAW_VEL_MAX_ERROR_SUM = 4000.0f;
    static constexpr float YAW_VEL_MAX_OUTPUT = 30000.0f;
    static constexpr float YAW_VEL_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_VEL_R_DERIVATIVE_KALMAN = 0.0f;
    static constexpr float YAW_VEL_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float YAW_VEL_R_PROPORTIONAL_KALMAN = 1.7f;
    static constexpr float YAW_VEL_DEADZONE = 1.0f;

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

    static constexpr float PITCH_VEL_P = 200.0f;
    static constexpr float PITCH_VEL_I = 0.0f;
    static constexpr float PITCH_VEL_D = 0.0f;
    static constexpr float PITCH_VEL_MAX_ERROR_SUM = 10000.0f;
    static constexpr float PITCH_VEL_MAX_OUTPUT = 30000.0f;
    static constexpr float PITCH_VEL_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float PITCH_VEL_R_DERIVATIVE_KALMAN = 0.0f;
    static constexpr float PITCH_VEL_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float PITCH_VEL_R_PROPORTIONAL_KALMAN = 13.0f;
    static constexpr float PITCH_VEL_DEADZONE = 1.0f;
#endif

    static constexpr float USER_YAW_INPUT_SCALAR = 1.0f;
    static constexpr float USER_PITCH_INPUT_SCALAR = 1.0f;

    tap::Drivers *drivers;

    TurretSubsystem *turretSubsystem;

    // Yaw related values
    tap::algorithms::ContiguousFloat worldFrameYawSetpoint;
    float chassisFrameInitImuYawAngle;

    uint32_t prevTime;

    // Pitch/yaw PID controllers
    tap::algorithms::SmoothPid yawPosPid;
    tap::algorithms::SmoothPid yawVelPid;
    tap::algorithms::SmoothPid pitchPosPid;
    tap::algorithms::SmoothPid pitchVelPid;
};  // class TurretWorldRelativeChassisImuCommand

}  // namespace control::turret

}  // namespace aruwsrc

#endif  // TURRET_WORLD_RELATIVE_CHASSIS_IMU_COMMAND_HPP_
