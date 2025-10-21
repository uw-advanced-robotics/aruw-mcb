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

namespace aruwsrc::communication::serial
{
RttTelemetry::RttTelemetry(tap::Drivers* drivers)
    : drivers(drivers),
      rtt(0),  // Use RTT channel 0
      rttDevice(rtt),
      rttStream(rttDevice),
      periodicTimer(100),  // 100ms periodic updates for testing (was 1000)
      messageCounter(0)
{
}

void RttTelemetry::initialize()
{
    // RTT is automatically initialized by the modm framework
    // Send initialization message with robot name
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

    // Blink LED to indicate RTT telemetry is initializing
    if (drivers) {
        // Flash all LEDs in sequence to show initialization is happening
        for (int i = 0; i < 5; i++) {
            drivers->leds.set(tap::gpio::Leds::Green, false); // On
            drivers->leds.set(tap::gpio::Leds::Red, false);   // On
            // Small delay (not ideal but for debugging)
            for (volatile int j = 0; j < 500000; j++);
            drivers->leds.set(tap::gpio::Leds::Green, true);  // Off
            drivers->leds.set(tap::gpio::Leds::Red, true);    // Off
            for (volatile int j = 0; j < 500000; j++);
        }
        // Leave green on permanently to show we completed initialization
        drivers->leds.set(tap::gpio::Leds::Green, false);
    }

    // Try multiple simple writes to test RTT functionality
    const char* testStr1 = "RTT_INIT_TEST\n";
    const char* testStr2 = "HELLO_RTT_WORLD\n";
    const char* testStr3 = "==== RTT DEBUG START ====\n";
    
    // Write to SEGGER RTT for better compatibility
    writeToSeggerRTT(testStr3);
    writeToSeggerRTT(testStr1);
    writeToSeggerRTT(testStr2);
    
    // Also try the original modm RTT for comparison
    rtt.write(reinterpret_cast<const uint8_t*>(testStr3), strlen(testStr3));
    rtt.write(reinterpret_cast<const uint8_t*>(testStr1), strlen(testStr1));
    rtt.write(reinterpret_cast<const uint8_t*>(testStr2), strlen(testStr2));
    
    // Send JSON initialization message
    rttStream << "{\"type\":\"init\",\"timestamp\":" << getTimestamp() 
              << ",\"robot\":\"" << robotName << "\",\"message\":\"RTT Telemetry Initialized\"}" 
              << modm::endl;
              
    // Send additional debug info to both RTT systems
    char debugMsg[128];
    int debugLen = snprintf(debugMsg, sizeof(debugMsg), 
                           "RTT Init: Robot=%s, Time=%lu\n", 
                           robotName, getTimestamp());
    
    // Write to SEGGER RTT
    writeToSeggerRTT(debugMsg);
    
    // Write to modm RTT
    rtt.write(reinterpret_cast<const uint8_t*>(debugMsg), debugLen);
}

void RttTelemetry::sendAboutInfo(
    const char* robotName,
    const char* lastUser,
    const char* lastSha,
    const char* lastDate,
    const char* branchName)
{
    if (!isReady()) return;

    rttStream << "{\"type\":\"about\",\"timestamp\":" << getTimestamp() << ",\"data\":{"
              << "\"robotName\":\"" << robotName << "\","
              << "\"lastUser\":\"" << lastUser << "\","
              << "\"lastSha\":\"" << lastSha << "\","
              << "\"lastDate\":\"" << lastDate << "\","
              << "\"branchName\":\"" << branchName << "\""
              << "}}" << modm::endl;
}

void RttTelemetry::sendLogMessage(const char* level, const char* message)
{
    if (!isReady()) return;

    rttStream << "{\"type\":\"log\",\"timestamp\":" << getTimestamp() << ",\"level\":\"" << level
              << "\",\"message\":\"" << message << "\"}" << modm::endl;
}

size_t RttTelemetry::sendRawData(const uint8_t* data, size_t length)
{
    return rtt.write(data, length);
}

bool RttTelemetry::isReady() const
{
    // Check if there's space in the transmit buffer
    // Use a simple heuristic - if we can write at least one byte, we're ready
    return true; // For now, always assume ready to avoid const cast issues
}

void RttTelemetry::update()
{
    // Check for incoming RTT data from host
    uint8_t receivedByte;
    if (rtt.read(receivedByte)) {
        // Process received data - change LED pattern based on input
        if (drivers) {
            if (receivedByte == '0') {
                // Fast blinking pattern for '0' - use LED A for bidirectional feedback
                static uint32_t fastBlinkCounter = 0;
                fastBlinkCounter++;
                bool fastBlink = (fastBlinkCounter % 5) < 3; // Fast blink every 50ms
                drivers->leds.set(tap::gpio::Leds::A, fastBlink);
            } else if (receivedByte == '1') {
                // Solid LED A for '1'
                drivers->leds.set(tap::gpio::Leds::A, true);
            } else {
                // Turn off LED A for other characters
                drivers->leds.set(tap::gpio::Leds::A, false);
            }
        }
        
        // Echo back the received character
        char echoMsg[32];
        int echoLen = snprintf(echoMsg, sizeof(echoMsg), "ECHO: %c (0x%02X)\n", 
                              (receivedByte >= 32 && receivedByte <= 126) ? receivedByte : '?', 
                              receivedByte);
        rtt.write(reinterpret_cast<const uint8_t*>(echoMsg), echoLen);
    }

    if (periodicTimer.execute())
    {
        // Toggle LED to show RTT update is running - make it very obvious
        if (drivers) {
            static bool ledState = false;
            ledState = !ledState;
            drivers->leds.set(tap::gpio::Leds::Red, ledState);
            // Also toggle LED A to make it super obvious
            drivers->leds.set(tap::gpio::Leds::A, ledState);
        }

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

        // Send a simple test message first
        char testMsg[64];
        int len = snprintf(testMsg, sizeof(testMsg), "HEARTBEAT_%lu from %s\n", 
                          messageCounter, robotName);
        
        // Write to both RTT systems
        writeToSeggerRTT(testMsg);
        // rtt.write(reinterpret_cast<const uint8_t*>(testMsg), len);

        // Send periodic heartbeat with robot name and system information
        rttStream << "{\"type\":\"heartbeat\",\"timestamp\":" << getTimestamp()
                  << ",\"counter\":" << messageCounter++ 
                  << ",\"robot\":\"" << robotName << "\",\"data\":{"
                  << "\"uptime\":" << tap::arch::clock::getTimeMilliseconds()
                  << "}}" << modm::endl;
    }
}

uint32_t RttTelemetry::getTimestamp() const { return tap::arch::clock::getTimeMilliseconds(); }

void RttTelemetry::sendFormattedMessage(const char* type, const char* data)
{
    if (!isReady()) return;

    rttStream << "{\"type\":\"" << type << "\",\"timestamp\":" << getTimestamp() << ",\"data\":\""
              << data << "\"}" << modm::endl;
}

}  // namespace aruwsrc::communication::serial