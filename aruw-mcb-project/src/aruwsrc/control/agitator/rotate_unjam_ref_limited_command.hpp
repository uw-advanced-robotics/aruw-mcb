/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ROTATE_UNJAM_REF_LIMITED_COMMAND_HPP_
#define ROTATE_UNJAM_REF_LIMITED_COMMAND_HPP_

#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"
#include "tap/control/setpoint/commands/unjam_integral_command.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::agitator
{
/**
 * A command that will attempt to rotate an agitator a set amount and unjam if it
 * encounters a jam. This command has the option to be heat limited (in-game "heat")
 */
class RotateUnjamRefLimitedCommand
    : public tap::control::setpoint::MoveUnjamIntegralComprisedCommand
{
public:
    /**
     * @note: All parameters except for and `heatLimitBuffer` are passed directly to the
     * `tap::control::setpoint::MoveUnjamIntegralComprisedCommand` constructor, so see that class
     * for what those parameters do.
     *
     * @param[in] heatLimitBuffer If current_barrel_heat + heatLimitBuffer > barrel_heat_limit then
     * command will not be scheduled. i.e.: How close you can get to the heat limit before the
     * command won't be scheduled.
     */
    RotateUnjamRefLimitedCommand(
        aruwsrc::Drivers &drivers,
        tap::control::setpoint::IntegrableSetpointSubsystem &subsystem,
        tap::control::setpoint::MoveIntegralCommand &moveIntegralCommand,
        tap::control::setpoint::UnjamIntegralCommand &unjamCommand,
        const tap::communication::serial::RefSerialData::Rx::MechanismID turretID,
        const uint16_t heatLimitBuffer);

    bool isReady() override;

    bool isFinished() const override;

private:
    aruwsrc::Drivers &drivers;

    const tap::communication::serial::RefSerialData::Rx::MechanismID turretID;
    const uint16_t heatLimitBuffer;
};

}  // namespace aruwsrc::agitator

#endif  // ROTATE_UNJAM_REF_LIMITED_COMMAND_HPP_
