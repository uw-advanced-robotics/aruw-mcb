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

#include "rtt_telemetry.hpp"

#include <cstdio>
#include <cstring>

#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"

// Add SEGGER RTT support for better J-Link compatibility
extern "C" {
    typedef struct {
        const char* sName;
        char* pBuffer;
        unsigned int SizeOfBuffer;
        unsigned int WrOff;
        volatile unsigned int RdOff;
        unsigned int Flags;
    } SEGGER_RTT_BUFFER_UP;

    typedef struct {
        const char* sName;
        char* pBuffer;
        unsigned int SizeOfBuffer;
        volatile unsigned int WrOff;
        unsigned int RdOff;
        unsigned int Flags;
    } SEGGER_RTT_BUFFER_DOWN;

    typedef struct {
        char acID[16];
        int MaxNumUpBuffers;
        int MaxNumDownBuffers;
        SEGGER_RTT_BUFFER_UP aUp[2];
        SEGGER_RTT_BUFFER_DOWN aDown[2];
    } SEGGER_RTT_CB;

    static char _acUpBuffer[1024];
    static char _acDownBuffer[256];

    SEGGER_RTT_CB _SEGGER_RTT = {
        "SEGGER RTT",
        2,
        2,
        {
            { "Terminal", _acUpBuffer, sizeof(_acUpBuffer), 0, 0, 0 },
            { NULL, NULL, 0, 0, 0, 0 }
        },
        {
            { "Terminal", _acDownBuffer, sizeof(_acDownBuffer), 0, 0, 0 },
            { NULL, NULL, 0, 0, 0, 0 }
        }
    };
}

// Helper function to write to SEGGER RTT
static void writeToSeggerRTT(const char* str) {
    extern SEGGER_RTT_CB _SEGGER_RTT;
    size_t len = strlen(str);
    auto& buffer = _SEGGER_RTT.aUp[0];
    for (size_t i = 0; i < len; i++) {
        unsigned int wrOff = buffer.WrOff;
        unsigned int nextWrOff = (wrOff + 1) % buffer.SizeOfBuffer;
        if (nextWrOff != buffer.RdOff) {
            buffer.pBuffer[wrOff] = str[i];
            buffer.WrOff = nextWrOff;
        }
    }
}

// Helper function to read from SEGGER RTT
static bool readFromSeggerRTT(uint8_t& data) {
    extern SEGGER_RTT_CB _SEGGER_RTT;
    auto& buffer = _SEGGER_RTT.aDown[0];
    
    if (buffer.RdOff == buffer.WrOff) {
        return false; // Buffer empty
    }
    
    data = static_cast<uint8_t>(buffer.pBuffer[buffer.RdOff]);
    buffer.RdOff = (buffer.RdOff + 1) % buffer.SizeOfBuffer;
    return true;
}

namespace aruwsrc::communication::serial
{
RttTelemetry::RttTelemetry(tap::Drivers* drivers)
    : drivers(drivers),
      periodicTimer(1000),  // 1 second periodic heartbeat
      ledBlinkTimer(500),   // 500ms LED blink rate
      messageCounter(0),
      firstInputReceived(false)
{
}

void RttTelemetry::initialize()
{
    // Get robot name
#if defined(TARGET_DRONE)
    const char* robotName = "TARGET_DRONE";
#elif defined(TARGET_ENGINEER)
    const char* robotName = "TARGET_ENGINEER";
#elif defined(TARGET_SENTRY_ECLIPSE)
    const char* robotName = "TARGET_SENTRY_ECLIPSE";
#elif defined(TARGET_HERO_ZERO)
    const char* robotName = "TARGET_HERO_ZERO";
#elif defined(TARGET_STANDARD_NULL)
    const char* robotName = "TARGET_STANDARD_NULL";
#elif defined(TARGET_STANDARD_VOID)
    const char* robotName = "TARGET_STANDARD_VOID";
#else
    const char* robotName = "TARGET_UNKNOWN";
#endif

    // Send simple initialization message
    char initMsg[128];
    snprintf(initMsg, sizeof(initMsg), 
             "{\"type\":\"init\",\"timestamp\":%lu,\"robot\":\"%s\"}\n",
             getTimestamp(), robotName);
    writeToSeggerRTT(initMsg);
}

void RttTelemetry::update()
{
    // Check for incoming RTT data from host
    uint8_t receivedByte;
    if (readFromSeggerRTT(receivedByte)) {
        // First input received - change LED pattern to red blinking
        if (!firstInputReceived) {
            firstInputReceived = true;
        }
        
        // Echo back the received character
        char echoMsg[32];
        snprintf(echoMsg, sizeof(echoMsg), "ECHO: %c (0x%02X)\n", 
                (receivedByte >= 32 && receivedByte <= 126) ? receivedByte : '?', 
                receivedByte);
        writeToSeggerRTT(echoMsg);
    }

    // Handle LED patterns
    if (drivers && firstInputReceived) {
        // Red blinking pattern after first input received
        if (ledBlinkTimer.execute()) {
            static bool redLedState = false;
            redLedState = !redLedState;
            drivers->leds.set(tap::gpio::Leds::Red, !redLedState); // Inverted logic
        }
    }

    // Send periodic heartbeat
    if (periodicTimer.execute())
    {
        // Get robot name
#if defined(TARGET_DRONE)
        const char* robotName = "TARGET_DRONE";
#elif defined(TARGET_ENGINEER)
        const char* robotName = "TARGET_ENGINEER";
#elif defined(TARGET_SENTRY_ECLIPSE)
        const char* robotName = "TARGET_SENTRY_ECLIPSE";
#elif defined(TARGET_HERO_ZERO)
        const char* robotName = "TARGET_HERO_ZERO";
#elif defined(TARGET_STANDARD_NULL)
        const char* robotName = "TARGET_STANDARD_NULL";
#elif defined(TARGET_STANDARD_VOID)
        const char* robotName = "TARGET_STANDARD_VOID";
#else
        const char* robotName = "TARGET_UNKNOWN";
#endif

        // Send simple heartbeat with robot info
        char heartbeat[256];
        snprintf(heartbeat, sizeof(heartbeat),
                "{\"type\":\"heartbeat\",\"timestamp\":%lu,\"counter\":%lu,\"robot\":\"%s\",\"uptime\":%lu}\n",
                getTimestamp(), messageCounter++, robotName, tap::arch::clock::getTimeMilliseconds());
        writeToSeggerRTT(heartbeat);
    }
}

uint32_t RttTelemetry::getTimestamp() const { return tap::arch::clock::getTimeMilliseconds(); }

}  // namespace aruwsrc::communication::serial