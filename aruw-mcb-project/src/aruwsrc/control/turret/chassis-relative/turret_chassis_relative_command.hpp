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
#include "tap/control/turret/turret_subsystem_interface.hpp"

namespace tap
{
class Drivers;
}

namespace aruwsrc::control::turret
{
/**
 * A command that contorls specified turret relative to the chassis.
 */
class TurretChassisRelativeCommand : public tap::control::Command
{
public:
    TurretChassisRelativeCommand(
        tap::Drivers *drivers,
        tap::control::turret::TurretSubsystemInterface *turretSubsystem);

    bool isReady() override;

    bool isFinished() const override;

    const char *getName() const override { return "Turret CR command"; }

    void initialize() override;

    void execute() override;

    void end(bool) override;

private:
#if defined(TARGET_SOLDIER)
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

    static constexpr float USER_YAW_INPUT_SCALAR = 1.0f;
    static constexpr float USER_PITCH_INPUT_SCALAR = 1.0f;

    tap::Drivers *drivers;
    tap::control::turret::TurretSubsystemInterface *turretSubsystem;

    uint32_t prevTime = 0;

    // Pitch/yaw PID controllers
    tap::algorithms::SmoothPid yawPid;
    tap::algorithms::SmoothPid pitchPid;
};
}  // namespace aruwsrc::control::turret

#endif  // TURRET_CHASSIS_RELATIVE_COMMAND_HPP_
