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

#include "aruwsrc/communication/rtt/create_rtt_error.hpp"
#include "aruwsrc/communication/rtt/segger_rtt_wrapper.hpp"

namespace
{
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

            logMessageProcessing = now <= logMessageDeadlineMillis;
            errorMessageProcessing = now <= errorMessageDeadlineMillis;

            bool activelySendingTelemetry =
                (!messageQueue.isEmpty() || !printQueue.isEmpty()) || firstInputReceived;
            bool recentRttInput = activelySendingTelemetry && now <= messageIndicatorDeadlineMillis;

            // 0 = none, 1 = one active, 2 = both active
            connectionState =
                static_cast<ConnectionState>(activelySendingTelemetry + recentRttInput);

            ledAnimator.update(
                drivers,
                connectionState,
                logMessageProcessing,
                errorMessageProcessing,
                now);

            // In Ozone mode (idle, no messages received yet), don't send telemetry
            // heartbeat info, only prints and errors are allowed
            ozoneMode = !firstInputReceived;

            logHeartbeatInfo();
            sendQueuedMessages(ozoneMode);
        }

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

    uint32_t now = tap::arch::clock::getTimeMilliseconds();
    logMessageDeadlineMillis = now + MESSAGE_DURATION;
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

    uint32_t now = tap::arch::clock::getTimeMilliseconds();
    errorMessageDeadlineMillis = now + MESSAGE_DURATION;
}

bool RttTelemetry::ensureSpaceOrClearQueue(
    modm::BoundedDeque<QueuedMessage, MAX_QUEUED_MESSAGES>& queue,
    std::size_t requiredSpace,
    std::size_t available,
    std::size_t currentSize,
    const char* queueName)
{
    if (currentSize + requiredSpace > available)
    {
        std::size_t destroyed = 0;
        while (!queue.isEmpty())
        {
            queue.removeBack();
            ++destroyed;
        }
        std::string err = std::string("Out of memory.") + "Av: " + std::to_string(available) +
                          ", req: " + std::to_string(requiredSpace) +
                          ", buf use: " + std::to_string(currentSize) +
                          ". Del: " + std::to_string(destroyed) + ". Cleared " + queueName;
        RAISE_ERROR(drivers, this, err.c_str());
        return false;
    }
    return true;
}

void RttTelemetry::appendEvents(
    std::string& out,
    modm::BoundedDeque<QueuedMessage, MAX_QUEUED_MESSAGES>& queue,
    const char* label,
    std::size_t available)
{
    out += '"';
    out.append(label);
    out += '"';
    out += ":[";
    while (!queue.isEmpty())
    {
        const auto& msg = queue.getFront();

        // Calculate space needed: quotes + escaped content + potential comma
        std::size_t extra = 2;  // Opening and closing quotes
        for (size_t i = 0; i < msg.length; i++)
        {
            extra++;
            if (msg.data[i] == '"' || msg.data[i] == '\\')
            {
                extra++;  // Extra char for escape
            }
        }

        if (!ensureSpaceOrClearQueue(queue, extra + 2, available, out.size(), label))
        {
            break;
        }

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
        if (!queue.isEmpty())
        {
            out += ',';
        }
    }
    out += ']';
}

void RttTelemetry::sendQueuedMessages(bool ozone)
{
    // Send only if there are any messages, errors, or prints to send
    if (messageQueue.isEmpty() && errorQueue.isEmpty() && printQueue.isEmpty())
    {
        return;
    }

    const std::size_t available = aruwsrc::communication::rtt::seggerRttGetAvailWriteSpace();
    // Require space for at least "{}\\n" plus one payload char before building a line.
    if (available <= rttLineOverhead)
    {
        std::string err = std::string("Insufficient RTT space available (") +
                          std::to_string(available) + " bytes). Cannot send queued messages.";
        RAISE_ERROR(drivers, this, err.c_str());
        return;
    }

    std::string out;
    out.reserve(available);
    out += '{';
    bool first = true;
    while (!messageQueue.isEmpty())
    {
        // Get from back to ensure we send the heartbeat
        const auto& msg = messageQueue.getBack();
        const std::size_t extra = (first ? 0 : 1) + msg.length;

        if (!ensureSpaceOrClearQueue(messageQueue, extra + 2, available, out.size(), "message"))
        {
            break;
        }

        messageQueue.removeBack();
        if (!first)
        {
            out += ',';
        }
        first = false;
        out.append(msg.data, msg.length);
        if (ozone)
        {
            // After sending timestamp, break to avoid flooding in ozone mode
            break;
        }
    }
    out += ',';
    appendEvents(out, errorQueue, "_ERROR_", available);
    out += ',';
    appendEvents(out, printQueue, "_PRINT_", available);

    out += "}\n";

    if (!writeRttLine(out))
    {
        RAISE_ERROR(drivers, this, "Failed to write RTT telemetry line.");
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
    logSignal("dt_us", dt);
    logSignal("robot", robotName);
    logSignal("time", time);
}
}  // namespace aruwsrc::communication::rtt
