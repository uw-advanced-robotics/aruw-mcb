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

#include "aruwsrc/communication/rtt/segger_rtt_wrapper.hpp"

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
      printQueue(),
      errorQueue()
{
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
            if (!firstInputReceived)
            {
                firstInputReceived = true;
            }
            messageIndicatorDeadlineMillis =
                tap::arch::clock::getTimeMilliseconds() + MESSAGE_INDICATOR_MS;

            // Echo back the received character
            char echoMsg[32];
            std::snprintf(
                echoMsg,
                sizeof(echoMsg),
                "ECHO: %c (0x%02X)",
                (receivedByte >= 32 && receivedByte <= 126) ? receivedByte : '?',
                receivedByte);
            println(echoMsg);
        }

        // Handle LED patterns for A-H row
        // 1. If RTT message received within last 1s -> bidirectional bounce (two-way)
        // 2. Else if actively sending telemetry -> unidirectional sweep (one-way, no heartbeat)
        // 3. Else -> slow group flash (idle, not sending/receiving) ((this also happens when
        // connected to ozone,
        //    since it never sends the initial RTT input))
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
    ledAnimator.notifyPrintLogged(tap::arch::clock::getTimeMilliseconds());
}

void RttTelemetry::queueErrorMessage(const char* message)
{
    if (errorQueue.isFull())
    {
        // Drop oldest message to make room
        errorQueue.removeFront();
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
    errorQueue.append(msg);
    ledAnimator.notifyErrorLogged(tap::arch::clock::getTimeMilliseconds());
}

void RttTelemetry::appendEvents(
    std::string& out,
    modm::BoundedDeque<QueuedMessage, MAX_QUEUED_MESSAGES>& queue,
    const char* label) const
{
    out += '{';
    out += label;
    out += ":[";
    while (!queue.isEmpty())
    {
        const auto& msg = queue.getFront();
        queue.removeFront();

        out += '"';
        for (size_t i = 0; i < msg.length; i++)
        {
            char c = msg.data[i];
            if (c == '"' || c == '\\')
            {
                out += '\\';
            }
            out += c;
        }
        out += '"';
    }
}

void RttTelemetry::sendQueuedMessages()
{
    if (messageQueue.isEmpty())
    {
        return;
    }

    const std::size_t available = aruwsrc::communication::rtt::seggerRttGetAvailWriteSpace();
    // Require space for at least "{}\\n" plus one payload char before building a line.
    if (available <= kRttLineOverhead)
    {
        logError("not enough avail");
        return;
    }

    std::string out;
    out.reserve(available);
    out += '{';
    bool first = true;
    while (!messageQueue.isEmpty())
    {
        const auto& msg = messageQueue.getFront();
        messageQueue.removeFront();
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
    }

    appendEvents(out, errorQueue, "_ERROR_");
    appendEvents(out, printQueue, "_PRINT_");

    out += "}\n";

    if (!writeRttLine(out))
    {
        logError("wth");
    }
}

void RttTelemetry::logHeartbeatInfo()
{
    const char* robotName;
// this is stupid
#if defined(TARGET_STANDARD_NULL)
    robotName = "TARGET_STANDARD_NULL";
#elif defined(TARGET_STANDARD_VOID)
    robotName = "TARGET_STANDARD_VOID";
#elif defined(TARGET_DRONE)
    robotName = "TARGET_DRONE";
#elif defined(TARGET_ENGINEER)
    robotName = "TARGET_ENGINEER";
#elif defined(TARGET_ENGI_2025)
    robotName = "TARGET_ENGI_2025";
#elif defined(TARGET_SENTRY_ECLIPSE)
    robotName = "TARGET_SENTRY_ECLIPSE";
#elif defined(TARGET_HERO_ZERO)
    robotName = "TARGET_HERO_ZERO";
#elif defined(TARGET_DART)
    robotName = "TARGET_DART";
#elif defined(TARGET_TESTBED)
    robotName = "TARGET_TESTBED";
#elif defined(TARGET_BLANK)
    robotName = "TARGET_BLANK";
#elif defined(TARGET_MOTOR_TESTER)
    robotName = "TARGET_MOTOR_TESTER";
#elif defined(TARGET_LAUNCHER_TARGET)
    robotName = "TARGET_LAUNCHER_TARGET";
#elif defined(TARGET_CHARACTERIZER)
    robotName = "TARGET_CHARACTERIZER";
#else
    robotName = "TARGET_UNKNOWN";
#endif

    static uint32_t lastLoopTime = 0;
    uint32_t currentTime = tap::arch::clock::getTimeMicroseconds();
    uint32_t dt = currentTime - lastLoopTime;
    lastLoopTime = currentTime;

    // Convert currentTime to seconds
    float time = currentTime / 1000.0f;
    logSignal("time", time);
    logSignal("dt_us", dt);
    logSignal("robot", robotName);
}

}  // namespace aruwsrc::communication::rtt
