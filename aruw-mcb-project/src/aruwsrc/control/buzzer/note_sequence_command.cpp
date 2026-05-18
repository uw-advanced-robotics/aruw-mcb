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

#include "note_sequence_command.hpp"

namespace aruwsrc::control::buzzer
{
NoteSequenceCommand::NoteSequenceCommand(
    BuzzerSubsystem& buzzer,
    const std::span<const uint8_t> notes,
    const uint16_t noteLengthMillis)
    : buzzer(buzzer),
      notes(notes),
      noteLengthMillis(noteLengthMillis)
{
    addSubsystemRequirement(&buzzer);
}

void NoteSequenceCommand::initialize() { startTime = tap::arch::clock::getTimeMilliseconds(); }

void NoteSequenceCommand::execute()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();

    currNoteIndex = ((currTime - startTime) / noteLengthMillis);
    uint8_t newNote = notes[currNoteIndex];

    if (isFinished() || newNote == currNote) return;

    buzzer.playNote(newNote);
    currNote = newNote;
}

}  // namespace aruwsrc::control::buzzer