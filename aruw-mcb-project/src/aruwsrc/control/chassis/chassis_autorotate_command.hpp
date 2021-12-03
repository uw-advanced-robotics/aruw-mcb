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

#ifndef CHASSIS_AUTOROTATE_COMMAND_HPP_
#define CHASSIS_AUTOROTATE_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/control/turret/turret_subsystem_interface.hpp"

namespace tap
{
class Drivers;
}

namespace aruwsrc
{
namespace control
{
namespace chassis
{
class ChassisSubsystem;

/**
 * A command that continuously attempts to rotate the chasis so that the turret is
 * aligned with the center of the chassis.
 */
class ChassisAutorotateCommand : public tap::control::Command
{
public:
    static constexpr float CHASSIS_AUTOROTATE_PID_KP = -100.0f;
    static constexpr float SETPOINT_AND_CURRENT_YAW_MATCH_THRESHOLD = 1.0f;

    ChassisAutorotateCommand(
        tap::Drivers* drivers,
        ChassisSubsystem* chassis,
        const tap::control::turret::TurretSubsystemInterface* turret,
        bool chassisFrontBackIdentical = false);

    void initialize() override;

    /**
     * Uses a PD controller to calculate the desired chassis rotation RPM based on the
     * difference between the turret angle and the center of the chassis, then
     * applies the desired rotation along with user desired <x, y> components to the
     * chassis subsystem's `setDesiredRpm` function.
     */
    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "chassis autorotate"; }

private:
    tap::Drivers* drivers;
    ChassisSubsystem* chassis;
    const tap::control::turret::TurretSubsystemInterface* turret;
    /**
     * If the front and back of the chassis may be treated as the same entities.
     * This only matters if your turret can spin 360 degrees and will allow the
     * autorotate to recenter either around the front or back of the chassis.
     */
    bool chassisFrontBackIdentical;
    /**
     * `true` if the chassis is currently actually autorotating, `false` otherwise
     * (in which case on rotation may happen). Autorotation may not happen if the
     * user requests a user input that moves the turret from the front of the chassis
     * to the back. If the chassis front and back are identical, then there is no
     * reason to autorotate until the turret is done turning around.
     */
    bool chassisAutorotating;

    void updateAutorotateState(const tap::control::turret::TurretSubsystemInterface* turret);
};  // class ChassisAutorotateCommand

}  // namespace chassis

}  // namespace control

}  // namespace aruwsrc

#endif  // CHASSIS_AUTOROTATE_COMMAND_HPP_
