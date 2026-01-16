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
#ifndef CLEMENTINE_SEGGER_RTT_HPP_
#define CLEMENTINE_SEGGER_RTT_HPP_

#include <cstddef>
#include <cstdint>
#include <cstdarg>

namespace aruwsrc::communication::rtt
{
enum class RttWriteMode : uint8_t
{
    NoBlockSkip = 0,
    NoBlockTrim = 1,
    BlockIfFull = 2,
};

namespace clementine_rtt
{
struct BufferUp
{
    const char* sName;
    char* pBuffer;
    unsigned SizeOfBuffer;
    unsigned WrOff;
    unsigned RdOff;
    unsigned Flags;
};

struct BufferDown
{
    const char* sName;
    char* pBuffer;
    unsigned SizeOfBuffer;
    unsigned WrOff;
    unsigned RdOff;
    unsigned Flags;
};

struct ControlBlock
{
    char acID[16];
    int MaxNumUpBuffers;
    int MaxNumDownBuffers;
    BufferUp aUp[2];
    BufferDown aDown[2];
};

static char g_upBuffer[1024];
static char g_downBuffer[256];

static ControlBlock g_rtt = {
    "SEGGER RTT",
    2,
    2,
    {{"Terminal", g_upBuffer, sizeof(g_upBuffer), 0, 0, 0}, {nullptr, nullptr, 0, 0, 0, 0}},
    {{"Terminal", g_downBuffer, sizeof(g_downBuffer), 0, 0, 0}, {nullptr, nullptr, 0, 0, 0, 0}}};

inline unsigned getAvailWriteSpace()
{
    const auto& ring = g_rtt.aUp[0];
    const unsigned rdOff = ring.RdOff;
    const unsigned wrOff = ring.WrOff;
    if (rdOff <= wrOff)
    {
        return ring.SizeOfBuffer - (wrOff - rdOff) - 1u;
    }
    return (rdOff - wrOff) - 1u;
}

inline std::size_t write(const uint8_t* data, std::size_t length)
{
    if (!data || length == 0)
    {
        return 0;
    }

    auto& buffer = g_rtt.aUp[0];
    std::size_t written = 0;
    for (std::size_t i = 0; i < length; ++i)
    {
        unsigned wrOff = buffer.WrOff;
        unsigned nextWrOff = (wrOff + 1) % buffer.SizeOfBuffer;
        if (nextWrOff == buffer.RdOff)
        {
            break;
        }
        buffer.pBuffer[wrOff] = static_cast<char>(data[i]);
        buffer.WrOff = nextWrOff;
        ++written;
    }
    return written;
}

inline bool read(uint8_t& data)
{
    auto& buffer = g_rtt.aDown[0];
    if (buffer.RdOff == buffer.WrOff)
    {
        return false;
    }
    data = static_cast<uint8_t>(buffer.pBuffer[buffer.RdOff]);
    buffer.RdOff = (buffer.RdOff + 1) % buffer.SizeOfBuffer;
    return true;
}
}  // namespace clementine_rtt

inline void seggerRttInit() {}
inline void seggerRttSetUpMode(RttWriteMode) {}
inline std::size_t seggerRttGetAvailWriteSpace()
{
    return clementine_rtt::getAvailWriteSpace();
}
inline std::size_t seggerRttWrite(const uint8_t* data, std::size_t length)
{
    return clementine_rtt::write(data, length);
}
inline std::size_t seggerRttWriteWithMode(const uint8_t* data, std::size_t length, RttWriteMode)
{
    return clementine_rtt::write(data, length);
}
inline bool seggerRttRead(uint8_t& data)
{
    return clementine_rtt::read(data);
}
inline int seggerRttPrintf(const char*, ...)
{
    return -1;
}
inline int seggerRttVprintf(const char*, va_list*)
{
    return -1;
}

}  // namespace aruwsrc::communication::rtt

#endif  // CLEMENTINE_SEGGER_RTT_HPP_
