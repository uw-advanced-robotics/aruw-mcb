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

#include <cstdarg>

#include "tap/architecture/clock.hpp"

// #define ARUWSRC_RTT_USE_CLEMENTINE

#if defined(ARUWSRC_RTT_USE_CLEMENTINE)
#include "aruwsrc/communication/rtt/clementine_segger_rtt.hpp"
#else
#include "aruwsrc/communication/rtt/segger_rtt_wrapper.hpp"
#endif

namespace
{
constexpr std::size_t kRttLineOverhead = 3;  // "{", "}\n"

std::size_t escapedLength(const char* data, std::size_t length)
{
    std::size_t extra = 0;
    for (std::size_t i = 0; i < length; ++i)
    {
        if (data[i] == '"' || data[i] == '\\') extra++;
    }
    return length + extra;
}

bool writeRttLine(const std::string& line)
{
    // RTT modes: Skip drops if full, Trim sends partial, Block waits for space.
    // Telemetry uses Skip so JSON lines are either complete or not sent.
    auto written = aruwsrc::communication::rtt::seggerRttWriteWithMode(
        reinterpret_cast<const uint8_t*>(line.data()),
        line.size(),
        aruwsrc::communication::rtt::RttWriteMode::NoBlockSkip);
    return written == line.size();
}
}  // namespace

namespace aruwsrc::communication::rtt
{
RttTelemetry::RttTelemetry(tap::Drivers* drivers)
    : modm::pt::Protothread(),
      drivers(drivers),
      messageIndicatorDeadlineMillis(0),
      ledAnimator(),
      messageCounter(0),
      firstInputReceived(false),
      messageQueue(),
      printQueue()
{
}

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

bool RttTelemetry::updateTelemetryAsync()
{
    PT_BEGIN();

    while (true)
    {
        // Process incoming RTT data from host
        uint8_t receivedByte;
        if (aruwsrc::communication::rtt::seggerRttRead(receivedByte))
        {
            // First input received - change LED pattern to red blinking
            if (!firstInputReceived)
            {
                firstInputReceived = true;
            }

            // Turn off red LED for a short indicator period to show message receipt
            messageIndicatorDeadlineMillis =
                tap::arch::clock::getTimeMilliseconds() + MESSAGE_INDICATOR_MS;

            // Echo back the received character
            char echoMsg[32];
            std::snprintf(
                echoMsg,
                sizeof(echoMsg),
                "ECHO: %c (0x%02X)\n",
                (receivedByte >= 32 && receivedByte <= 126) ? receivedByte : '?',
                receivedByte);
            queueMessage(echoMsg);
        }

        // Handle LED patterns for A-H row
        // 1. If RTT message received within last 1s -> bidirectional bounce (two-way)
        // 2. Else if actively sending telemetry -> unidirectional sweep (one-way, no heartbeat)
        // 3. Else -> slow group flash (idle, not sending/receiving)
        {
            uint32_t now = tap::arch::clock::getTimeMilliseconds();
            bool activelySendingTelemetry =
                (!messageQueue.isEmpty() || !printQueue.isEmpty()) || firstInputReceived;
            bool recentRttInput = activelySendingTelemetry && now <= messageIndicatorDeadlineMillis;
            ledAnimator.update(drivers, activelySendingTelemetry, recentRttInput, now);
        }

        logHeartbeatInfo();
        sendQueuedMessages();

        // Yield to allow other protothreads to run
        PT_YIELD();
    }

    PT_END();
}

void RttTelemetry::queueMessage(const char* message)
{
    // Check if queue is full
    if (messageQueue.isFull())
    {
        // Drop oldest message to make room
        messageQueue.removeFront();
    }

    // Create new message
    QueuedMessage msg;
    size_t len = std::snprintf(msg.data, MAX_MESSAGE_SIZE, "%s", message);

    // std::snprintf returns the number of characters that would have been written
    // Clamp to actual buffer size
    if (len >= MAX_MESSAGE_SIZE)
    {
        len = MAX_MESSAGE_SIZE - 1;  // Null terminator is already handled by std::snprintf
    }

    msg.length = len;

    // Add message to queue
    messageQueue.append(msg);
}

void RttTelemetry::queuePrintMessage(const char* message)
{
    // Check if queue is full
    if (printQueue.isFull())
    {
        // Drop oldest message to make room
        printQueue.removeFront();
    }

    // Create new message
    QueuedMessage msg;
    size_t len = std::snprintf(msg.data, MAX_MESSAGE_SIZE, "%s", message);

    // std::snprintf returns the number of characters that would have been written
    // Clamp to actual buffer size
    if (len >= MAX_MESSAGE_SIZE)
    {
        len = MAX_MESSAGE_SIZE - 1;  // Null terminator is already handled by std::snprintf
    }

    msg.length = len;

    // Add message to queue
    printQueue.append(msg);
}

void RttTelemetry::sendQueuedMessages()
{
    // Send print messages first
    while (!printQueue.isEmpty())
    {
        const auto& msg = printQueue.getFront();
        const std::size_t available = aruwsrc::communication::rtt::seggerRttGetAvailWriteSpace();
        const std::size_t needed =
            sizeof("{\"print\":\"") - 1 + escapedLength(msg.data, msg.length) + 3;
        // Only emit full JSON lines; partial lines break the host parser.
        if (available < needed)
        {
            break;
        }

        std::string out;
        out.reserve(needed);
        out += "{\"print\":\"";
        for (size_t i = 0; i < msg.length; i++)
        {
            char c = msg.data[i];
            if (c == '"' || c == '\\')
            {
                out += '\\';
            }
            out += c;
        }
        out += "\"}\n";

        if (!writeRttLine(out))
        {
            break;
        }

        printQueue.removeFront();
    }

    if (messageQueue.isEmpty())
    {
        return;
    }

    const std::size_t available = aruwsrc::communication::rtt::seggerRttGetAvailWriteSpace();
    // Require space for at least "{}\\n" plus one payload char before building a line.
    if (available < kRttLineOverhead + 1)
    {
        return;
    }

    std::string out;
    out.reserve(available);
    out += '{';
    bool first = true;
    std::size_t sendCount = 0;
    const auto queueSize = messageQueue.getSize();
    for (size_t i = 0; i < queueSize; ++i)
    {
        const auto& msg = messageQueue.get(static_cast<decltype(messageQueue)::Index>(i));
        const std::size_t extra = (first ? 0 : 1) + msg.length;
        if (out.size() + extra + 2 > available)
        {
            break;
        }
        if (!first)
        {
            out += ',';
        }
        first = false;
        out.append(msg.data, msg.length);
        ++sendCount;
    }

    if (sendCount == 0)
    {
        return;
    }

    out += "}\n";

    if (!writeRttLine(out))
    {
        return;
    }

    for (std::size_t i = 0; i < sendCount; ++i)
    {
        messageQueue.removeFront();
    }
}

void RttTelemetry::logHeartbeatInfo()
{
    const char* robotName;
#if defined(TARGET_DRONE)
    robotName = "TARGET_DRONE";
#elif defined(TARGET_ENGINEER)
    robotName = "TARGET_ENGINEER";
#elif defined(TARGET_SENTRY_ECLIPSE)
    robotName = "TARGET_SENTRY_ECLIPSE";
#elif defined(TARGET_HERO_ZERO)
    robotName = "TARGET_HERO_ZERO";
#elif defined(TARGET_STANDARD_NULL)
    robotName = "TARGET_STANDARD_NULL";
#elif defined(TARGET_STANDARD_VOID)
    robotName = "TARGET_STANDARD_VOID";
#elif defined(TARGET_MOTOR_TESTER)
    robotName = "TARGET_MOTOR_TESTER";
#else
    robotName = "TARGET_UNKNOWN";
#endif

    logSignal("time", tap::arch::clock::getTimeMilliseconds());
    logSignal("robot", robotName);
    logSignal("messageCount", messageCounter++);
}

}  // namespace aruwsrc::communication::rtt
