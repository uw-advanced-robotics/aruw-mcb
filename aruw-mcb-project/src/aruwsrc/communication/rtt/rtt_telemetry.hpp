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

#ifndef RTT_TELEMETRY_HPP_
#define RTT_TELEMETRY_HPP_

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <type_traits>

#include "tap/util_macros.hpp"

#include "modm/container/deque.hpp"
#include "modm/processing/protothread.hpp"

#include "aruwsrc/communication/rtt/rtt_led_animator.hpp"
// Forward declarations
namespace tap
{
class Drivers;
}  // namespace tap

namespace aruwsrc::communication::rtt
{
/**
 * RTT (Real Time Transfer) telemetry handler for sending debug and diagnostic
 * information to the host through J-Link RTT protocol without halting the target.
 *
 * Uses modm protothreads for asynchronous, non-blocking operation to prevent
 * telemetry transmission from interfering with real-time robot control.
 *
 * Provides heartbeat functionality, input detection, and comprehensive logging
 * of robot subsystem data.
 */
class RttTelemetry : public modm::pt::Protothread
{
public:
    /**
     * Constructor
     * @param drivers Pointer to the global drivers instance
     */
    RttTelemetry(tap::Drivers* drivers);

    /**
     * Asynchronous telemetry update using modm protothreads.
     *
     * @return false when protothread completes (which never happens as this runs in infinite loop)
     */
    mockable bool updateTelemetryAsync();

    template <typename T>
    void logSignal(const char* label, const T& value)
    {
        emit_scalar_json_and_queue(label, value);
    }

    template <typename T, size_t N>
    void logSignal(const char* label, const T (&values)[N])
    {
        if constexpr (N == 1)
            emit_scalar_json_and_queue(label, values[0]);
        else
            emit_array_json_and_queue(label, values);
    }

    template <typename T, typename... Ts>
    void logSignal(const char* label, T v0, Ts... rest)
    {
        constexpr size_t N = sizeof...(rest) + 1;
        T vals[N] = {v0, rest...};
        logSignal(label, vals);
    }

    template <typename... Args>
    void println(const char* first, Args... rest)
    {
        std::string msg;
        msg.reserve(MAX_MESSAGE_SIZE);

        auto append = [&](const char* s) {
            if (s) msg += s;
        };

        (append(first), ..., append(rest));

        queuePrintMessage(msg.c_str());
    }

    int printf(const char* format, ...);

    /**
     * Blocking function that's already called by protothread, so no need to call manually. Only
     * public for use by `modm_abort()`.
     */
    mockable void sendQueuedMessages();

#if !defined(ENV_UNIT_TESTS) || !defined(PLATFORM_HOSTED)
private:
#endif
    tap::Drivers* drivers;

    // Deadline (ms) until which the message indicator keeps the row animation active
    uint32_t messageIndicatorDeadlineMillis;
    static constexpr uint32_t MESSAGE_INDICATOR_MS = 1000;  // milliseconds (1s)

    RttLedAnimator ledAnimator;

    // Total messages sent
    uint32_t messageCounter;

    // Flag to track if we've received first RTT input
    bool firstInputReceived;

    // Message queue for asynchronous transmission
    static constexpr size_t MAX_QUEUED_MESSAGES = 64;
    static constexpr size_t MAX_MESSAGE_SIZE = 64;

    struct QueuedMessage
    {
        char data[MAX_MESSAGE_SIZE];
        size_t length;

        QueuedMessage() : data{0}, length(0) {}
    };

    modm::BoundedDeque<QueuedMessage, MAX_QUEUED_MESSAGES> messageQueue;

    // Separate queue for println() messages to avoid interference with JSON telemetry
    modm::BoundedDeque<QueuedMessage, MAX_QUEUED_MESSAGES> printQueue;

    /**
     * Queue a message for asynchronous transmission (JSON telemetry)
     */
    void queueMessage(const char* message);

    /**
     * Queue a print message for asynchronous transmission (separate from JSON telemetry)
     */
    void queuePrintMessage(const char* message);

    /**
     * Queue timestamp, robot type, and counter
     */
    void logHeartbeatInfo();

    template <class T>
    void append_json_value(std::string& out, const T& v)
    {
        if constexpr (std::is_floating_point_v<T>)
        {
            char buf[64];
            int vq = static_cast<int32_t>(v * 1000);
            if (vq < 0)
            {
                std::snprintf(
                    buf,
                    sizeof(buf),
                    "-%d.%03d",
                    std::abs(vq) / 1000,
                    std::abs(vq) % 1000);
            }
            else
            {
                std::snprintf(buf, sizeof(buf), "%d.%03d", vq / 1000, vq % 1000);
            }
            out += buf;
        }
        else if constexpr (std::is_integral_v<T> && !std::is_same_v<T, bool>)
        {
            out += std::to_string(static_cast<long long>(v));
        }
        else if constexpr (std::is_same_v<T, bool>)
        {
            out += (v ? "true" : "false");
        }
        else if constexpr (std::is_convertible_v<T, const char*>)
        {
            out += '"';
            for (const char* p = static_cast<const char*>(v); *p; ++p)
            {
                if (*p == '"' || *p == '\\') out += '\\';
                out += *p;
            }
            out += '"';
        }
        else
        {
            static_assert(std::is_arithmetic_v<T>, "Unsupported type for JSON logging");
        }
    }

    template <typename T>
    void emit_scalar_json_and_queue(const char* label, const T& v)
    {
        std::string msg;
        msg.reserve(64);
        msg += "\"";
        msg += label;
        msg += "\":";
        append_json_value(msg, v);
        queueMessage(msg.c_str());
    }

    template <typename T, size_t N>
    void emit_array_json_and_queue(const char* label, const T (&arr)[N])
    {
        std::string msg;
        msg.reserve(32 + N * 16);
        msg += "\"";
        msg += label;
        msg += "\":[";

        for (size_t i = 0; i < N; ++i)
        {
            if (i) msg += ',';
            append_json_value(msg, arr[i]);
        }
        msg += "]";
        queueMessage(msg.c_str());
    }
};

}  // namespace aruwsrc::communication::rtt

#endif  // RTT_TELEMETRY_HPP_
