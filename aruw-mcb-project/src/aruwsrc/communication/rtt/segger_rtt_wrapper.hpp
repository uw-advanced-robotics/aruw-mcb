/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef SEGGER_RTT_WRAPPER_HPP_
#define SEGGER_RTT_WRAPPER_HPP_

#include <cstdarg>
#include <cstddef>
#include <cstdint>

namespace aruwsrc::communication::rtt
{
enum class RttWriteMode : uint8_t
{
    NoBlockSkip = 0,
    NoBlockTrim = 1,
    BlockIfFull = 2,
};

void seggerRttInit();
void seggerRttSetUpMode(RttWriteMode mode);
std::size_t seggerRttGetAvailWriteSpace();
std::size_t seggerRttWrite(const uint8_t* data, std::size_t length);
std::size_t seggerRttWriteWithMode(const uint8_t* data, std::size_t length, RttWriteMode mode);
bool seggerRttRead(uint8_t& data);
int seggerRttPrintf(const char* format, ...);
int seggerRttVprintf(const char* format, va_list* args);

}  // namespace aruwsrc::communication::rtt

#endif  // SEGGER_RTT_HPP_
