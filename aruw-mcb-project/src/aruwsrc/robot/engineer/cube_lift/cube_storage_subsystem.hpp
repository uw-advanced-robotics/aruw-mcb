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
#include "aruwsrc/robot/engineer/arm/limit_switch_setpoint_interface.hpp"
#include "aruwsrc/robot/engineer/cube_lift/engineer_lift_constants.hpp"
#include "aruwsrc/robot/engineer/engineer_drivers.hpp"

using namespace aruwsrc::engineer;

namespace aruwsrc::robot::engineer
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
        float epsilon = 0.5f);

    void initialize() override;

    void setDesiredOutput(int16_t power) override;

    void refresh() override;

    void refreshSafeDisconnect() override;

    void moveTowardLowerBound();

    const char* getName() const override { return "Cube Storage"; }

    float getPosition() override;

protected:
    tap::motor::MotorInterface& motor;

    void stopDuringHoming() override;

    void setHome(uint64_t encoderPosition) override;

private:
    float lastTime = 0;
    float motorDesiredOutput = 0;
    int16_t homingOutput = 1000;
    uint64_t home = 0;
    tap::algorithms::SmoothPid pid;
    tap::algorithms::SmoothPid homingPID;
    // float velocitySetpoint = 100;

    float motorPos = 0;
    float pidOutput = 0;
};  // class CUBE_STORAGE

}  // namespace aruwsrc::robot::engineer
#endif  // CUBE_STORAGE_SUBSYSTEM_HPP_
