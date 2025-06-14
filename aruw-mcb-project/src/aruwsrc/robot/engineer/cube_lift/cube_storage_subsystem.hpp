/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef CUBE_STORAGE_SUBSYSTEM_HPP_
#define CUBE_STORAGE_SUBSYSTEM_HPP_

#include <tap/motor/motor_interface.hpp>

#include "tap/control/subsystem.hpp"

#include "aruwsrc/control/bounded-subsystem/one_sided_bounded_subsystem_interface.hpp"
#include "aruwsrc/control/bounded-subsystem/trigger/trigger_interface.hpp"
#include "aruwsrc/robot/engineer/engineer_drivers.hpp"
#include "aruwsrc/robot/engineer/limit_switch_setpoint_interface.hpp"

using namespace aruwsrc::engineer;

namespace aruwsrc::engineer::lift
{
class CubeStorageSubsystem : public LimitSwitchSetpointInterface
{
public:
    CubeStorageSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorInterface& storageLiftMotor,
        const tap::algorithms::SmoothPidConfig& configPos,
        const tap::algorithms::SmoothPidConfig& configHoming,
        aruwsrc::control::TriggerInterface& trigger,
        float home,
        float radius = 1.0f,
        float kS = 0,
        float epsilon = 0.5f);

    void initialize() override;

    void setDesiredOutput(int16_t power) override;

    void resetEncoderValue() override;

    float getEncoderValue() override;

    float getEncoderVelocity() override;

    const char* getName() const override { return "Cube Storage"; }

private:
    tap::motor::MotorInterface& motor;
    tap::algorithms::SmoothPid homingPID;
    int16_t homingOutput = 1000;
};  // class CUBE_STORAGE

}  // namespace aruwsrc::engineer::lift
#endif  // CUBE_STORAGE_SUBSYSTEM_HPP_
