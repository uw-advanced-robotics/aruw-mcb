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

#ifndef NOTE_SEQUENCES_HPP_
#define NOTE_SEQUENCES_HPP_

#include <stdint.h>

#include <array>

namespace aruwsrc::control::buzzer
{
static constexpr uint32_t MARIO_MUSHROOM_NOTE_LENGTH_MS = 34;
static constexpr std::array<uint8_t, 27> MARIO_MUSHROOM_NOTES{{37, 32, 37, 41, 44, 49, 44, 33, 37,
                                                               40, 45, 40, 45, 49, 52, 57, 52, 35,
                                                               39, 42, 47, 42, 47, 51, 54, 59, 54}};

static constexpr uint32_t MARIO_1UP_NOTE_LENGTH_MS = 130;
static constexpr std::array<uint8_t, 6> MARIO_1UP_NOTES{{41, 44, 53, 49, 51, 56}};

// https://onlinesequencer.net/3348282#t16
static constexpr uint32_t MARIO_PIPE_NOTE_LENGTH_MS = 20;
static constexpr std::array<uint8_t, 48> MARIO_PIPE_NOTES{
    {53, 46, 39, 53, 46, 39, 32, 25, 18, 11, 0, 0, 0, 0, 0, 0,
     53, 46, 39, 53, 46, 39, 32, 25, 18, 11, 0, 0, 0, 0, 0, 0,
     53, 46, 39, 53, 46, 39, 32, 25, 18, 11, 0, 0, 0, 0, 0, 0}};

static constexpr uint32_t ZELDA_SECRET_NOTE_LENGTH_MS = 130;
static constexpr std::array<uint8_t, 8> ZELDA_SECRET_NOTES{{44, 43, 40, 34, 33, 41, 45, 49}};

static constexpr uint32_t MEGALOVANIA_NOTE_LENGTH_MS = 125;
static constexpr std::array<uint8_t, 16> MEGALOVANIA_NOTES{
    {27, 27, 39, 0, 34, 0, 0, 33, 0, 32, 0, 30, 30, 27, 30, 32}};

static constexpr uint32_t SEVEN_NATION_ARMY_NOTE_LENGTH_MS = 121;
static constexpr std::array<uint8_t, 32> SEVEN_NATION_ARMY_NOTES{
    {27, 27, 27, 27, 27, 0,  27, 27, 30, 0,  0,  27, 0,  0,  25, 0,
     23, 23, 23, 23, 23, 23, 0,  0,  22, 22, 22, 22, 22, 22, 0,  0}};

static constexpr uint32_t HES_A_PIRATE_NOTE_LENGTH_MS = 145;
static constexpr std::array<uint8_t, 96> HES_A_PIRATE_NOTES{{
    27, 30, 32, 0, 32, 0, 32, 34, 35, 0,  35, 0, 35, 37, 34, 0, 34, 0, 32, 30, 30, 32, 0, 0,
    27, 30, 32, 0, 32, 0, 32, 34, 35, 0,  35, 0, 35, 37, 34, 0, 34, 0, 32, 30, 32, 0,  0, 0,
    27, 30, 32, 0, 32, 0, 32, 35, 37, 0,  37, 0, 37, 39, 40, 0, 40, 0, 39, 37, 39, 32, 0, 0,
    32, 34, 35, 0, 35, 0, 37, 0,  39, 32, 0,  0, 32, 35, 34, 0, 34, 0, 35, 32, 34, 0,  0, 0,
}};

static constexpr uint32_t IMU_CALIBRATE_SUCCESS_NOTE_LENGTH_MS = MARIO_MUSHROOM_NOTE_LENGTH_MS;
static constexpr auto& IMU_CALIBRATE_SUCCESS_NOTES = MARIO_MUSHROOM_NOTES;

static constexpr uint32_t IMU_CALIBRATE_FAIL_NOTE_LENGTH_MS = MARIO_PIPE_NOTE_LENGTH_MS;
static constexpr auto& IMU_CALIBRATE_FAIL_NOTES = MARIO_PIPE_NOTES;
}  // namespace aruwsrc::control::buzzer

#endif  // NOTE_SEQUENCES_HPP_
