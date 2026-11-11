/*
 * Copyright (c) 2023-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef DAMIAO_ROTATE_SLOT_COMMAND_
#define DAMIAO_ROTATE_SLOT_COMMAND_

#include <cstdint>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/command.hpp"

#include "aruwsrc/control/motor/damiao_subsystem.hpp"

namespace aruwsrc::motor_tester
{
class DamiaoRotateSlotCommand : public tap::control::Command
{
public:
    DamiaoRotateSlotCommand(control::motor::DamiaoSubsystem& subsystem);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;
    const char* getName() const override { return "Damiao rotate slot"; }

private:
    control::motor::DamiaoSubsystem& subsystem;

    static constexpr int SLOT_COUNT = 6;
    static constexpr float ROTATE_INCR = (2.0f * M_PI) / SLOT_COUNT;
    static constexpr float POSITION_TOLERANCE = 0.005f;
    tap::algorithms::SmoothPid positionPid{{
        .kp = 500.0f,
        .ki = 200.0f,
        .kd = 150.0f,
        .maxICumulative = 500.0f,
        .maxOutput = control::motor::DM_J4340_LIMITS.maxOutputMilliNm,
    }};
    uint32_t lastUpdateMillis = 0;
    float setpoint = 0.0f;
    float positionError = 0.0f;
};
}  // namespace aruwsrc::motor_tester

#endif  // DAMIAO_ROTATE_SLOT_COMMAND_
