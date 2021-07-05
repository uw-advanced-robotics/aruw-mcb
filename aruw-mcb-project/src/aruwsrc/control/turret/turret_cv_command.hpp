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

#ifndef TURRET_CV_COMMAND_HPP_
#define TURRET_CV_COMMAND_HPP_

#include "aruwlib/algorithms/contiguous_float.hpp"
#include "aruwlib/algorithms/smooth_pid.hpp"
#include "aruwlib/architecture/timeout.hpp"
#include "aruwlib/control/command.hpp"

#include "aruwsrc/control/chassis/chassis_subsystem.hpp"

#include "turret_subsystem.hpp"

namespace aruwsrc::control::turret
{
/**
 * A command that receives input from the vision system via the `XavierSerial` driver and aims the
 * turret accordingly using a position PID controller.
 */
class TurretCVCommand : public aruwlib::control::Command
{
public:
    /**
     * @param[in] drivers
     * @param[in] subsystem
     * @param[in] turretStartAngle
     * @param[in] yawPidConfig
     * @param[in] pitchPidConfig
     */
    TurretCVCommand(
        aruwlib::Drivers *drivers,
        TurretSubsystem *subsystem,
        float turretStartAngle,
        const aruwlib::algorithms::PidConfigStruct &yawPidConfig,
        const aruwlib::algorithms::PidConfigStruct &pitchPidConfig);

    void initialize() override;

    bool isFinished() const override { return false; }

    void execute() override;

    void end(bool) override;

    const char *getName() const override { return "turret cv"; }

private:
    aruwlib::Drivers *drivers;

    TurretSubsystem *turretSubsystem;

    aruwlib::algorithms::ContiguousFloat yawTargetAngle;
    aruwlib::algorithms::ContiguousFloat pitchTargetAngle;

    aruwlib::algorithms::SmoothPid yawPid;
    aruwlib::algorithms::SmoothPid pitchPid;

    uint32_t prevTime;

    void runYawPositionController(float dt);

    void runPitchPositionController(float dt);
};  // class TurretCvCommand

}  // namespace aruwsrc::control::turret

#endif  // TURRET_CV_COMMAND_HPP_
