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

static constexpr uint32_t UNKNOWN_RM_THEME_NOTE_LENGTH_MS = 800;
static constexpr std::array<uint8_t, 16> UNKNOWN_RM_THEME_NOTES{
    {34, 42, 42, 42, 34, 39, 39, 39, 35, 44, 44, 44, 39, 41, 41, 41}};

static constexpr uint32_t SUMMONING_GLORY_NOTE_LENGTH_MS = 215;
static constexpr std::array<uint8_t, 64> SUMMONING_GLORY_NOTES{{
    37, 37, 37, 37, 37, 39, 39, 40, 40, 40, 40, 40, 37, 37, 37, 37, 35, 35, 35, 35, 35, 37,
    35, 34, 34, 34, 34, 34, 30, 30, 30, 30, 37, 37, 37, 37, 37, 39, 39, 40, 40, 40, 40, 35,
    40, 40, 42, 42, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 0,  0,  0,  0,
}};

static constexpr uint32_t FUNK_IN_KINGDOM_NOTE_LENGTH_MS = 130;
static constexpr std::array<uint8_t, 32> FUNK_IN_KINGDOM_NOTES{
    {34, 0,  37, 0, 41, 0, 34, 32, 34, 0,  37, 41, 0,  0,  0,  34,
     0,  34, 37, 0, 41, 0, 37, 0,  39, 39, 39, 0,  37, 37, 37, 0}};

static constexpr uint32_t BUMBLEBEE_NOTE_LENGTH_MS = 83;
static constexpr std::array<uint8_t, 895> BUMBLEBEE_NOTES = {
    {53, 52, 51, 50, 51, 50, 49, 48, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 39, 38, 37, 36,
     37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 27, 26, 25, 24, 29, 28, 27, 26, 27, 26, 25, 24,
     29, 28, 28, 26, 25, 24, 23, 22, 21, 22, 23, 24, 25, 26, 27, 28, 29, 28, 27, 26, 25, 30, 29, 28,
     29, 28, 27, 26, 25, 26, 27, 28, 29, 28, 27, 26, 25, 30, 29, 28, 29, 28, 27, 26, 25, 26, 27, 28,
     29, 28, 27, 26, 27, 26, 25, 24, 25, 26, 27, 28, 29, 30, 29, 28, 29, 28, 27, 26, 27, 26, 25, 24,
     25, 26, 27, 28, 29, 31, 32, 33, 34, 33, 32, 31, 30, 35, 34, 33, 32, 33, 32, 31, 30, 31, 32, 33,
     34, 33, 32, 31, 30, 35, 34, 33, 34, 33, 32, 31, 30, 31, 32, 33, 34, 33, 32, 31, 32, 31, 30, 29,
     30, 31, 32, 33, 34, 35, 34, 33, 34, 33, 32, 31, 30, 29, 28, 29, 30, 31, 32, 33, 34, 35, 34, 33,
     34, 22, 22, 22, 22, 22, 22, 22, 23, 21, 23, 21, 23, 21, 23, 21, 22, 22, 22, 22, 22, 22, 22, 22,
     23, 21, 23, 21, 23, 21, 23, 21, 22, 23, 22, 21, 22, 23, 22, 21, 22, 23, 22, 21, 22, 23, 22, 21,
     22, 23, 24, 25, 26, 25, 24, 23, 22, 23, 24, 25, 26, 27, 28, 29, 27, 27, 27, 27, 27, 27, 27, 27,
     28, 26, 28, 26, 28, 26, 28, 26, 27, 27, 27, 27, 27, 27, 27, 27, 28, 26, 28, 26, 28, 26, 28, 26,
     27, 28, 27, 26, 27, 28, 27, 26, 27, 28, 27, 26, 27, 28, 27, 26, 27, 28, 29, 30, 31, 30, 29, 28,
     27, 28, 29, 30, 31, 32, 33, 34, 39, 38, 37, 36, 35, 40, 39, 38, 39, 38, 37, 36, 35, 36, 37, 38,
     39, 38, 37, 36, 37, 36, 35, 34, 35, 36, 37, 38, 39, 40, 41, 40, 41, 40, 39, 38, 39, 38, 37, 36,
     37, 36, 35, 34, 33, 32, 31, 30, 29, 30, 29, 28, 29, 30, 29, 28, 29, 30, 29, 28, 29, 30, 29, 28,
     29, 30, 29, 28, 29, 30, 29, 28, 29, 30, 29, 28, 29, 30, 29, 28, 28, 28, 28, 28, 41, 41, 37, 37,
     34, 34, 30, 30, 34, 34, 37, 37, 41, 41, 0,  41, 53, 53, 49, 49, 46, 46, 42, 42, 46, 46, 49, 49,
     53, 53, 0,  0,  0,  53, 41, 41, 0,  0,  0,  41, 29, 29, 0,  29, 29, 30, 31, 32, 33, 34, 35, 36,
     37, 38, 39, 40, 41, 40, 39, 38, 37, 42, 41, 40, 41, 40, 39, 38, 37, 38, 39, 40, 41, 40, 39, 38,
     37, 42, 41, 40, 41, 40, 39, 38, 37, 38, 39, 40, 41, 40, 39, 38, 39, 38, 37, 36, 37, 38, 39, 40,
     41, 42, 41, 40, 41, 40, 39, 38, 39, 38, 37, 36, 37, 38, 39, 40, 41, 43, 44, 45, 46, 45, 44, 43,
     42, 47, 46, 45, 46, 45, 44, 43, 42, 43, 44, 45, 46, 45, 44, 43, 42, 47, 46, 45, 46, 45, 44, 43,
     42, 43, 44, 45, 46, 45, 44, 43, 44, 43, 42, 41, 42, 43, 44, 45, 46, 47, 46, 45, 46, 45, 44, 43,
     41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 52, 51, 50, 49, 54, 53, 52, 53, 52, 51, 50,
     49, 50, 51, 52, 53, 52, 51, 50, 49, 54, 53, 52, 53, 52, 51, 50, 49, 50, 51, 52, 53, 53, 33, 34,
     35, 36, 37, 38, 39, 38, 37, 36, 37, 36, 35, 34, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 41, 40,
     41, 42, 41, 40, 41, 41, 33, 34, 35, 36, 37, 38, 39, 38, 37, 36, 37, 36, 35, 34, 33, 34, 35, 36,
     37, 38, 39, 40, 41, 43, 45, 46, 48, 49, 51, 52, 53, 52, 51, 50, 49, 54, 53, 52, 53, 52, 51, 50,
     49, 50, 51, 52, 53, 52, 51, 50, 49, 54, 53, 52, 53, 52, 51, 50, 49, 50, 51, 52, 53, 53, 33, 34,
     35, 36, 37, 38, 39, 38, 37, 36, 37, 36, 35, 34, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 41, 40,
     41, 42, 41, 40, 41, 41, 33, 34, 35, 36, 37, 38, 39, 38, 37, 36, 37, 36, 35, 34, 33, 34, 35, 36,
     37, 38, 39, 40, 41, 42, 41, 40, 41, 43, 44, 45, 46, 45, 44, 43, 44, 43, 42, 41, 42, 41, 40, 39,
     38, 37, 36, 35, 34, 33, 32, 31, 32, 31, 30, 29, 30, 29, 28, 27, 26, 25, 24, 23, 22, 23, 22, 21,
     22, 23, 22, 21, 22, 23, 22, 21, 22, 24, 25, 27, 29, 30, 29, 28, 29, 30, 29, 28, 29, 30, 29, 28,
     29, 31, 32, 33, 34, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 21, 22, 23, 24, 25, 26, 27, 28, 29,
     30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 46, 46, 0,  46, 53, 53, 0,  53, 34, 34, 0,  34,
     29, 29, 0,  29, 22, 22, 0}};

static constexpr uint32_t IMU_CALIBRATE_SUCCESS_NOTE_LENGTH_MS = MARIO_MUSHROOM_NOTE_LENGTH_MS;
static constexpr auto& IMU_CALIBRATE_SUCCESS_NOTES = MARIO_MUSHROOM_NOTES;

static constexpr uint32_t IMU_CALIBRATE_FAIL_NOTE_LENGTH_MS = MARIO_PIPE_NOTE_LENGTH_MS;
static constexpr auto& IMU_CALIBRATE_FAIL_NOTES = MARIO_PIPE_NOTES;

#ifdef TARGET_BALSTD
static constexpr uint32_t STATE_TRANSITION_FAIL_NOTE_LENGTH_MS = 100;
static constexpr std::array<uint8_t, 4> STATE_TRANSITION_FAIL_NOTES{{37, 0, 31, 0}};

static constexpr uint32_t WATCHDOG_INTERVENTION_NOTE_LENGTH_MS = 80;
static constexpr std::array<uint8_t, 8> WATCHDOG_INTERVENTION_NOTES{{52, 0, 52, 0, 52, 0, 52, 0}};

static constexpr uint32_t CHASSIS_OFFLINE_NOTE_LENGTH_MS = 200;
static constexpr std::array<uint8_t, 3> CHASSIS_OFFLINE_NOTES{{44, 37, 36}};
#endif
}  // namespace aruwsrc::control::buzzer

#endif  // NOTE_SEQUENCES_HPP_
