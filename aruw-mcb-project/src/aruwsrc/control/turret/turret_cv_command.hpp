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

#ifndef TURRET_CV_COMMAND_HPP_
#define TURRET_CV_COMMAND_HPP_

#include <aruwlib/algorithms/contiguous_float.hpp>
#include <aruwlib/architecture/timeout.hpp>
#include <aruwlib/control/command.hpp>

#include "aruwsrc/algorithms/turret_pid.hpp"
#include "aruwsrc/control/chassis/chassis_subsystem.hpp"

#include "turret_subsystem.hpp"

namespace aruwsrc
{
namespace turret
{
/**
 * A command that receives input from the vision system via the `XavierSerial` driver and aims the turret
 * accordingly.
 */
class TurretCVCommand : public aruwlib::control::Command
{
public:
    TurretCVCommand(aruwlib::Drivers *drivers, TurretSubsystem *subsystem);

    /**
     * Sends an initial tracking request, resets pitch and yaw PID.
     */
    void initialize() override;

    /**
     * @return `false` always.
     */
    bool isFinished() const override { return false; }

    /**
     * Attempts to acquire aim data from the vision system, setting the updated position if
     * Attempts to acquire aim data from the vision system, setting an updated desired position if
     * aquisition is successful. Also runs the position PID controller responsible for control of the turret
     * and sends an aim request every `TIME_BETWEEN_CV_REQUESTS`.
     */
    void execute() override;

    /**
     * Send a signal to the vision system to stop tracking the target.
     */
    void end(bool) override;

    const char *getName() const override { return "turret cv command"; }

private:
    static constexpr float YAW_P = 4500.0f;
    static constexpr float YAW_I = 0.0f;
    static constexpr float YAW_D = 140.0f;
    static constexpr float YAW_MAX_ERROR_SUM = 0.0f;
    static constexpr float YAW_MAX_OUTPUT = 32000.0f;
    static constexpr float YAW_Q_DERIVATIVE_KALMAN = 1.0f;
    static constexpr float YAW_R_DERIVATIVE_KALMAN = 20.0f;
    static constexpr float YAW_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float YAW_R_PROPORTIONAL_KALMAN = 0.0f;

    static constexpr float PITCH_P = 3500.0f;
    static constexpr float PITCH_I = 0.0f;
    static constexpr float PITCH_D = 80.0f;
    static constexpr float PITCH_MAX_ERROR_SUM = 0.0f;
    static constexpr float PITCH_MAX_OUTPUT = 32000.0f;
    static constexpr float PITCH_Q_DERIVATIVE_KALMAN = 1.5f;
    static constexpr float PITCH_R_DERIVATIVE_KALMAN = 20.0f;
    static constexpr float PITCH_Q_PROPORTIONAL_KALMAN = 1.0f;
    static constexpr float PITCH_R_PROPORTIONAL_KALMAN = 2.0f;

    static constexpr uint32_t TIME_BETWEEN_CV_REQUESTS = 1000;

    aruwlib::Drivers *drivers;

    TurretSubsystem *turretSubsystem;

    aruwlib::algorithms::ContiguousFloat yawTargetAngle;
    aruwlib::algorithms::ContiguousFloat pitchTargetAngle;

    aruwsrc::algorithms::TurretPid yawPid;
    aruwsrc::algorithms::TurretPid pitchPid;

    aruwlib::arch::MilliTimeout sendRequestTimer;

    void runYawPositionController();

    void runPitchPositionController();
};  // class TurretCvCommand

}  // namespace turret

}  // namespace aruwsrc

#endif  // TURRET_CV_COMMAND_HPP_
