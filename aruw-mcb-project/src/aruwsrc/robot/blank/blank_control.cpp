/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#if defined(TARGET_BLANK)

// TODO: import buzzer subsystem
#include "aruwsrc/control/buzzer/note_sequence_command.hpp"
#include "aruwsrc/control/buzzer/note_sequences.hpp"
#include "aruwsrc/drivers_singleton.hpp"
#include "aruwsrc/robot/blank/blank_drivers.hpp"
#include "aruwsrc/robot/robot_control.hpp"

using namespace aruwsrc::blank;
using namespace aruwsrc::control::buzzer;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
driversFunc drivers = DoNotUse_getDrivers;

namespace blank_control
{

// TODO: instantiate buzzer subsystem
// BuzzerSubsystem buzzer(...)

NoteSequenceCommand hesAPirateCommand(buzzer, HES_A_PIRATE_NOTES, HES_A_PIRATE_NOTE_LENGTH_MS);

BuzzerSubsystem buzzer(drivers());


void initializeSubsystems()
{
    // TODO: register initialize buzzer subsystem

    buzzer.setDefaultCommand(&hesAPirateCommand);
}

}  // namespace blank_control

namespace aruwsrc::blank
{
void initSubsystemCommands(aruwsrc::blank::Drivers *) { blank_control::initializeSubsystems(); }

}  // namespace aruwsrc::blank

#endif
