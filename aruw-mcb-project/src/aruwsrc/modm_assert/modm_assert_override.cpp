#include <cstdio>

#include "aruwsrc/communication/rtt/segger_rtt_wrapper.hpp"
#include "modm/architecture/interface/assert.hpp"
#include "modm/platform/core/heap_table.hpp"

// This replaces the weak implementation in libmodm.a
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
        "| Ctx: 0x%lx | Beh: 0x%02x !!!\r\n",
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