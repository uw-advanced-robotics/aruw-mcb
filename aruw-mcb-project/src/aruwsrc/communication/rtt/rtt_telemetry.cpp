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

#include "aruwsrc/build_info.hpp"
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

            ozoneMode = !firstInputReceived;
            if (ozoneMode)
            {
                logHeartbeatInfo();
            }
            else {
                logHeartbeatInfo();
                sendQueuedMessages();
            }
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
        while (!queue.isEmpty())
        {
            queue.removeBack();
        }
        // Use a static, zero-allocation error string
        RAISE_ERROR(drivers, this, "RTT Telemetry buffer full. Cleared queue.");
        return false;
    }
    return true;
}

void RttTelemetry::appendEvents(
    char* out_buf,
    std::size_t& out_len,
    modm::BoundedDeque<QueuedMessage, MAX_QUEUED_MESSAGES>& queue,
    const char* label,
    std::size_t available)
{
    auto append = [&](char c) {
        if (out_len < available) out_buf[out_len++] = c;
    };
    auto append_str = [&](const char* s) {
        while (*s) append(*s++);
    };

    append('"');
    append_str(label);
    append('"');
    append(':');
    append('[');

    while (!queue.isEmpty())
    {
        const auto& msg = queue.getFront();

        std::size_t extra = 2;
        for (size_t i = 0; i < msg.length; i++)
        {
            extra++;
            if (msg.data[i] == '"' || msg.data[i] == '\\') extra++;
        }

        if (!ensureSpaceOrClearQueue(queue, extra + 2, available, out_len, label))
        {
            break;
        }

        queue.removeFront();

        append('"');
        for (size_t i = 0; i < msg.length; i++)
        {
            char c = msg.data[i];
            if (c == '"' || c == '\\') append('\\');
            append(c);
        }
        append('"');

        if (!queue.isEmpty()) append(',');
    }
    append(']');
}

void RttTelemetry::sendQueuedMessages()
{
    if (messageQueue.isEmpty() && errorQueue.isEmpty() && printQueue.isEmpty()) return;

    const std::size_t available = aruwsrc::communication::rtt::seggerRttGetAvailWriteSpace();
    if (available <= rttLineOverhead)
    {
        RAISE_ERROR(drivers, this, "Insufficient RTT space available.");
        return;
    }

    // THE FIX: Static buffer in BSS RAM instead of dynamic heap allocation
    static char out_buf[2048];
    std::size_t out_len = 0;

    // Safety cap to prevent buffer overflow
    const std::size_t max_write = (available < sizeof(out_buf)) ? available : sizeof(out_buf);

    auto append = [&](char c) {
        if (out_len < max_write) out_buf[out_len++] = c;
    };

    append('{');
    bool first = true;
    while (!messageQueue.isEmpty())
    {
        const auto& msg = messageQueue.getBack();
        const std::size_t extra = (first ? 0 : 1) + msg.length;

        if (!ensureSpaceOrClearQueue(messageQueue, extra + 2, max_write, out_len, "message")) break;

        messageQueue.removeBack();
        if (!first) append(',');
        first = false;

        for (size_t i = 0; i < msg.length; ++i) append(msg.data[i]);
    }

    append(',');
    appendEvents(out_buf, out_len, errorQueue, "_ERROR_", max_write);
    append(',');
    appendEvents(out_buf, out_len, printQueue, "_PRINT_", max_write);
    append('}');
    append('\n');

    // Write the raw buffer directly
    auto written = aruwsrc::communication::rtt::seggerRttWriteWithMode(
        reinterpret_cast<const uint8_t*>(out_buf),
        out_len,
        aruwsrc::communication::rtt::RttWriteMode::NoBlockSkip);

    if (written != out_len)
    {
        RAISE_ERROR(drivers, this, "Failed to write RTT telemetry line.");
    }
}

void RttTelemetry::logHeartbeatInfo()
{
    const char* robotName = ROBOT_NAME;

    static uint32_t lastLoopTime = 0;
    uint32_t currentTime = tap::arch::clock::getTimeMicroseconds();
    uint32_t dt = currentTime - lastLoopTime;
    lastLoopTime = currentTime;

    // Convert currentTime to seconds
    float time = currentTime / 1000000.0f;
    logSignal("dt_us", dt);
    logSignal("robot", robotName);
    logSignal("time", time);
}
}  // namespace aruwsrc::communication::rtt
