/*
 * Copyright (c) 2020-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include <cstdio>

#include "aruwsrc/communication/rtt/segger_rtt_wrapper.hpp"
#include "modm/architecture/interface/assert.hpp"

// This replaces the weak implementation
extern "C" void modm_abandon(const modm::AssertionInfo &info)
{
    using namespace aruwsrc::communication::rtt;

    char logBuffer[128];
    int len;

    // Format the header
    len = snprintf(
        logBuffer,
        sizeof(logBuffer),
        "\r\n!!! ABANDON: %s ",
        info.name ? info.name : "Unknown");

    if (len > 0)
    {
        seggerRttWriteWithMode(
            reinterpret_cast<const uint8_t *>(logBuffer),
            static_cast<std::size_t>(len),
            RttWriteMode::BlockIfFull);
    }

    // Format and write description if enabled
#if MODM_ASSERTION_INFO_HAS_DESCRIPTION
    if (info.description)
    {
        len = snprintf(logBuffer, sizeof(logBuffer), "| Desc: %s ", info.description);
        if (len > 0)
        {
            seggerRttWriteWithMode(
                reinterpret_cast<const uint8_t *>(logBuffer),
                static_cast<std::size_t>(len),
                RttWriteMode::BlockIfFull);
        }
    }
#endif

    // Format and write Context/Behavior
    len = snprintf(
        logBuffer,
        sizeof(logBuffer),
        "| Context: 0x%lx | Behavior: 0x%02x !!!\r\n",
        (unsigned long)info.context,
        (unsigned int)info.behavior.value);

    if (len > 0)
    {
        seggerRttWriteWithMode(
            reinterpret_cast<const uint8_t *>(logBuffer),
            static_cast<std::size_t>(len),
            RttWriteMode::BlockIfFull);
    }
}