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
#ifndef BUZZER_NOTE_SEQUENCE_COMMAND_HPP_
#define BUZZER_NOTE_SEQUENCE_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "buzzer_subsystem.hpp"

namespace aruwsrc::control::buzzer
{

/**
 * Plays a sequence of constant length notes on a buzzer.
 */
class BuzzerNoteSequenceCommand : public tap::control::Command
{
public:
    BuzzerNoteSequenceCommand(
        BuzzerSubsystem& buzzer,
        const uint8_t* notes,
        const size_t numNotes,
        const uint16_t noteLengthMillis);

    void initialize() override;

    void execute() override;

    void end(bool) override { buzzer.stop(); }

    bool isFinished() const override { return noteIndex >= numNotes; }

    const char* getName() const override { return "Buzzer Note Sequence Command"; }

private:
    BuzzerSubsystem& buzzer;
    const uint8_t* notes;
    const size_t numNotes;
    const uint16_t noteLengthMillis;

    uint32_t startTime;
    size_t noteIndex;
};  // class BuzzerNoteSequenceCommand

}  // namespace aruwsrc::control::buzzer
#endif  // BUZZER_NOTE_SEQUENCE_COMMAND_HPP_
