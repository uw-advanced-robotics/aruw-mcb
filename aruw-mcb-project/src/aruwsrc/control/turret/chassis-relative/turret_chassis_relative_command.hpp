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

#ifndef TURRET_CHASSIS_RELATIVE_COMMAND_HPP_
#define TURRET_CHASSIS_RELATIVE_COMMAND_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/command.hpp"

#include "aruwsrc/control/turret/turret_subsystem_interface.hpp"
#include "aruwsrc/util_macros.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::turret
{
/**
 * A command that controls specified turret relative to the chassis.
 *
 * This command is a vessel that calls functions defined in `chassis_frame_turret_controller.hpp`
 * using the user input as the turret setpoint. This command contains the necessary variables to
 * call the functions `ChassisFrameTurretController::runPitchPidController` and
 * `ChassisFrameTurretController::runYawPidController`. This command will then just call these
 * functions, which will run the turret controller and command the turret subsystem to move to the
 * desired setpoint.
 */
class TurretChassisRelativeCommand : public tap::control::Command
{
public:
    TurretChassisRelativeCommand(
        aruwsrc::Drivers *drivers,
        TurretSubsystemInterface *turretSubsystem);

    bool isReady() override;

    const char *getName() const override { return "Turret CR command"; }

    void initialize() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool) override;

private:
#if defined(ALL_SOLDIERS)
    static constexpr float YAW_P = 4000.0f;
    static constexpr float YAW_I = 0.0f;
    static constexpr float YAW_D = 190.0f;
    static constexpr float YAW_MAX_ERROR_SUM = 0.0f;
    static constexpr float YAW_MAX_OUTPUT = 32000.0f;
    static constexpr float YAW_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_R_DERIVATIVE_KALMAN = 30.0f;
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
#elif defined(TARGET_SENTINEL)
    static constexpr float YAW_P = 4000.0f;
    static constexpr float YAW_I = 0.0f;
    static constexpr float YAW_D = 130.0f;
    static constexpr float YAW_MAX_ERROR_SUM = 0.0f;
    static constexpr float YAW_MAX_OUTPUT = 30000.0f;
    static constexpr float YAW_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_R_DERIVATIVE_KALMAN = 10.0f;
    static constexpr float YAW_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float YAW_R_PROPORTIONAL_KALMAN = 0.0f;

    static constexpr float PITCH_P = 3400.0f;
    static constexpr float PITCH_I = 0.0f;
    static constexpr float PITCH_D = 100.0f;
    static constexpr float PITCH_MAX_ERROR_SUM = 0.0f;
    static constexpr float PITCH_MAX_OUTPUT = 30000.0f;
    static constexpr float PITCH_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float PITCH_R_DERIVATIVE_KALMAN = 20.0f;
    static constexpr float PITCH_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float PITCH_R_PROPORTIONAL_KALMAN = 0.0f;
#else
    static constexpr float YAW_P = 4000.0f;
    static constexpr float YAW_I = 0.0f;
    static constexpr float YAW_D = 190.0f;
    static constexpr float YAW_MAX_ERROR_SUM = 0.0f;
    static constexpr float YAW_MAX_OUTPUT = 30000.0f;
    static constexpr float YAW_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_R_DERIVATIVE_KALMAN = 30.0f;
    static constexpr float YAW_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float YAW_R_PROPORTIONAL_KALMAN = 0.0f;

    static constexpr float PITCH_P = 4000.0f;
    static constexpr float PITCH_I = 0.0f;
    static constexpr float PITCH_D = 130.0f;
    static constexpr float PITCH_MAX_ERROR_SUM = 0.0f;
    static constexpr float PITCH_MAX_OUTPUT = 30000.0f;
    static constexpr float PITCH_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float PITCH_R_DERIVATIVE_KALMAN = 10.0f;
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
    TurretSubsystemInterface *turretSubsystem;

    uint32_t prevTime = 0;

    // Pitch/yaw PID controllers
    tap::algorithms::SmoothPid yawPid;
    tap::algorithms::SmoothPid pitchPid;
};
}  // namespace aruwsrc::control::turret

#endif  // TURRET_CHASSIS_RELATIVE_COMMAND_HPP_
