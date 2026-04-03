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

#include "segger_rtt_wrapper.hpp"

#include <algorithm>
#include <cstdarg>

extern "C"
{
#include "SEGGER_RTT.h"
#if defined(_WIN32) || defined(SEGGER_RTT_LOCK_EMBOS)
    void OS_SIM_EnterCriticalSection(void){};
    void OS_SIM_LeaveCriticalSection(void){};
#endif
}

namespace
{
const unsigned telemetryBufferIndex = 0;
const unsigned printfBufferIndex = 1;
const unsigned printfBufferSize = 256;

static char printfBuffer[printfBufferSize];
static bool printfBufferConfigured = false;
static aruwsrc::communication::rtt::RttWriteMode upMode =
    aruwsrc::communication::rtt::RttWriteMode::NoBlockSkip;
static bool initialized = false;

unsigned seggerMode(aruwsrc::communication::rtt::RttWriteMode mode)
{
    switch (mode)
    {
        case aruwsrc::communication::rtt::RttWriteMode::NoBlockSkip:
            return SEGGER_RTT_MODE_NO_BLOCK_SKIP;
        case aruwsrc::communication::rtt::RttWriteMode::NoBlockTrim:
            return SEGGER_RTT_MODE_NO_BLOCK_TRIM;
        case aruwsrc::communication::rtt::RttWriteMode::BlockIfFull:
            return SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL;
    }
    return SEGGER_RTT_MODE_NO_BLOCK_SKIP;
}

void ensurePrintfBufferConfigured()
{
    if (printfBufferConfigured)
    {
        return;
    }

    const int status = SEGGER_RTT_ConfigUpBuffer(
        printfBufferIndex,
        "Printf",
        printfBuffer,
        sizeof(printfBuffer),
        SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    if (status >= 0)
    {
        printfBufferConfigured = true;
    }
}

void ensureInitialized()
{
    if (initialized)
    {
        return;
    }

    SEGGER_RTT_Init();
    SEGGER_RTT_SetFlagsUpBuffer(telemetryBufferIndex, seggerMode(upMode));
    initialized = true;
}
}  // namespace

namespace aruwsrc::communication::rtt
{
void seggerRttInit() { ensureInitialized(); }

void seggerRttSetUpMode(RttWriteMode mode)
{
    ensureInitialized();
    upMode = mode;
    SEGGER_RTT_SetFlagsUpBuffer(telemetryBufferIndex, seggerMode(mode));
}

std::size_t seggerRttGetAvailWriteSpace()
{
    ensureInitialized();
    return SEGGER_RTT_GetAvailWriteSpace(telemetryBufferIndex);
}

std::size_t seggerRttWrite(const uint8_t* data, std::size_t length)
{
    ensureInitialized();
    if (!data || length == 0)
    {
        return 0;
    }

    return seggerRttWriteWithMode(data, length, upMode);
}

std::size_t seggerRttWriteWithMode(const uint8_t* data, std::size_t length, RttWriteMode mode)
{
    ensureInitialized();
    if (!data || length == 0)
    {
        return 0;
    }

    switch (mode)
    {
        case RttWriteMode::NoBlockSkip:
        {
            const unsigned prevFlags = seggerMode(upMode);
            if (prevFlags != SEGGER_RTT_MODE_NO_BLOCK_SKIP)
            {
                SEGGER_RTT_SetFlagsUpBuffer(telemetryBufferIndex, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
            }
            const std::size_t written =
                SEGGER_RTT_WriteNoLock(telemetryBufferIndex, data, static_cast<unsigned>(length));
            if (prevFlags != SEGGER_RTT_MODE_NO_BLOCK_SKIP)
            {
                SEGGER_RTT_SetFlagsUpBuffer(telemetryBufferIndex, prevFlags);
            }
            return written;
        }
        case RttWriteMode::NoBlockTrim:
        {
            const std::size_t avail = SEGGER_RTT_GetAvailWriteSpace(telemetryBufferIndex);
            const std::size_t toWrite = std::min(avail, length);
            if (toWrite == 0)
            {
                return 0;
            }
            return SEGGER_RTT_WriteNoLock(
                telemetryBufferIndex,
                data,
                static_cast<unsigned>(toWrite));
        }
        case RttWriteMode::BlockIfFull:
        {
            const unsigned prevFlags = seggerMode(upMode);
            if (prevFlags != SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL)
            {
                SEGGER_RTT_SetFlagsUpBuffer(
                    telemetryBufferIndex,
                    SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
            }
            const std::size_t written =
                SEGGER_RTT_WriteNoLock(telemetryBufferIndex, data, static_cast<unsigned>(length));
            if (prevFlags != SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL)
            {
                SEGGER_RTT_SetFlagsUpBuffer(telemetryBufferIndex, prevFlags);
            }
            return written;
        }
    }

    return 0;
}

bool seggerRttRead(uint8_t& data)
{
    ensureInitialized();
    return SEGGER_RTT_Read(telemetryBufferIndex, &data, 1u) == 1u;
}

int seggerRttPrintf(const char* format, ...)
{
    ensureInitialized();
    ensurePrintfBufferConfigured();
    if (!printfBufferConfigured)
    {
        return -1;
    }
    const unsigned bufferIndex = printfBufferIndex;

    va_list args;
    va_start(args, format);
    int result = SEGGER_RTT_vprintf(bufferIndex, format, &args);
    va_end(args);
    return result;
}

int seggerRttVprintf(const char* format, va_list* args)
{
    ensureInitialized();
    ensurePrintfBufferConfigured();
    if (!printfBufferConfigured)
    {
        return -1;
    }
    const unsigned bufferIndex = printfBufferIndex;

    return SEGGER_RTT_vprintf(bufferIndex, format, args);
}

// The printf funciton is unused in this current implementation.
// It is provided in the envent that future telemetry requirements
// need formatted output over RTT, but currently all telemetry
// is sent as JSON lines via the updateTelemetryAsync function, for parsing
// by Control Tower.

// Future work could re-enable this function if needed.
/*
// Paste into RttTelemetry class in rtt_telemetry.*:
int RttTelemetry::printf(const char* format, ...)
{
    if (!format)
    {
        return 0;
    }

    va_list args;
    va_start(args, format);
    int result = aruwsrc::communication::rtt::seggerRttVprintf(format, &args);
    va_end(args);
    return result;
}
*/

}  // namespace aruwsrc::communication::rtt
