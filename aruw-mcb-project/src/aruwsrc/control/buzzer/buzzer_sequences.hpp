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

#ifndef BUZZER_SEQUENCES_HPP_
#define BUZZER_SEQUENCES_HPP_

#include <stdint.h>

#include <cstddef>

namespace aruwsrc::control::buzzer
{
static constexpr uint32_t MARIO_MUSHROOM_NOTE_LENGTH_MS = 34;
static constexpr uint8_t MARIO_MUSHROOM_NOTES[]{37, 32, 37, 41, 44, 49, 44, 33, 37,
                                                40, 45, 40, 45, 49, 52, 57, 52, 35,
                                                39, 42, 47, 42, 47, 51, 54, 59, 54};

static constexpr uint32_t ZELDA_SECRET_NOTE_LENGTH_MS = 130;
static constexpr uint8_t ZELDA_SECRET_NOTES[]{44, 43, 40, 34, 33, 41, 45, 49};

}  // namespace aruwsrc::control::buzzer

#endif  // BUZZER_SEQUENCES_HPP_