/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TURRET_CHASSIS_RELATIVE_POSITION_COMMAND_HPP_
#define TURRET_CHASSIS_RELATIVE_POSITION_COMMAND_HPP_

#include <aruwlib/algorithms/contiguous_float.hpp>
#include <aruwlib/control/command.hpp>

#include "aruwsrc/algorithms/turret_pid.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"

namespace aruwsrc
{
namespace turret
{
class TurretChassisRelativePositionCommand : public aruwlib::control::Command
{
public:
    TurretChassisRelativePositionCommand(TurretSubsystem *subsystem, aruwlib::Drivers *drivers);

    void initialize() override;

    bool isFinished() const override { return false; }

    void execute() override;

    void end(bool) override;

    const char *getName() const override { return "turret chassis relative position command"; }

private:
#if defined(TARGET_SOLDIER)
    static constexpr float YAW_P = 4500.0f;
    static constexpr float YAW_I = 0.0f;
    static constexpr float YAW_D = 140.0f;
    static constexpr float YAW_MAX_ERROR_SUM = 0.0f;
    static constexpr float YAW_MAX_OUTPUT = 32000.0f;
    static constexpr float YAW_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_R_DERIVATIVE_KALMAN = 20.0f;
    static constexpr float YAW_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float YAW_R_PROPORTIONAL_KALMAN = 0.0f;

    static constexpr float PITCH_P = 4500.0f;
    static constexpr float PITCH_I = 0.0f;
    static constexpr float PITCH_D = 90.0f;
    static constexpr float PITCH_MAX_ERROR_SUM = 0.0f;
    static constexpr float PITCH_MAX_OUTPUT = 32000.0f;
    static constexpr float PITCH_Q_DERIVATIVE_KALMAN = 1.5f;
    static constexpr float PITCH_R_DERIVATIVE_KALMAN = 20.0f;
    static constexpr float PITCH_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float PITCH_R_PROPORTIONAL_KALMAN = 2.0f;

    static constexpr float USER_YAW_INPUT_SCALAR = 0.75f;
    static constexpr float USER_PITCH_INPUT_SCALAR = 0.5f;

    static constexpr float PITCH_GRAVITY_COMPENSATION_KP = 4000.0f;
#elif defined(TARGET_SENTINEL)
    static constexpr float YAW_P = 4500.0f;
    static constexpr float YAW_I = 0.0f;
    static constexpr float YAW_D = 140.0f;
    static constexpr float YAW_MAX_ERROR_SUM = 0.0f;
    static constexpr float YAW_MAX_OUTPUT = 32000.0f;
    static constexpr float YAW_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_R_DERIVATIVE_KALMAN = 20.0f;
    static constexpr float YAW_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float YAW_R_PROPORTIONAL_KALMAN = 0.0f;

    static constexpr float PITCH_P = 4500.0f;
    static constexpr float PITCH_I = 0.0f;
    static constexpr float PITCH_D = 90.0f;
    static constexpr float PITCH_MAX_ERROR_SUM = 0.0f;
    static constexpr float PITCH_MAX_OUTPUT = 32000.0f;
    static constexpr float PITCH_Q_DERIVATIVE_KALMAN = 1.5f;
    static constexpr float PITCH_R_DERIVATIVE_KALMAN = 20.0f;
    static constexpr float PITCH_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float PITCH_R_PROPORTIONAL_KALMAN = 2.0f;

    static constexpr float USER_YAW_INPUT_SCALAR = 0.75f;
    static constexpr float USER_PITCH_INPUT_SCALAR = 0.5f;

    static constexpr float PITCH_GRAVITY_COMPENSATION_KP = 0.0f;
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

    static constexpr float PITCH_GRAVITY_COMPENSATION_KP = 0.0f;
#endif

    TurretSubsystem *turretSubsystem;

    aruwsrc::algorithms::TurretPid yawPid;
    aruwsrc::algorithms::TurretPid pitchPid;

    aruwlib::Drivers *drivers;

    void runYawPositionController();
    void runPitchPositionController();
};

}  // namespace turret

}  // namespace aruwsrc

#endif  // TURRET_CHASSIS_RELATIVE_POSITION_COMMAND_HPP_
