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

#include <cstdio>
#include <cstring>
#include <string>
#include <type_traits>

#include "tap/architecture/periodic_timer.hpp"

#include "modm/math/geometry/vector2.hpp"
#include "modm/processing/protothread.hpp"

// Forward declarations
namespace tap
{
class Drivers;
namespace communication::serial
{
class RefSerial;
}
}  // namespace tap

namespace aruwsrc
{
namespace control
{
class ControlOperatorInterface;
}
namespace serial
{
class VisionCoprocessor;
}
}  // namespace aruwsrc

// SEGGER RTT headers
// extern "C" {
// #include "SEGGER_RTT.h"
// }

namespace tap
{
class Drivers;
}

namespace aruwsrc::communication::serial
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
    // template <typename T, const char* L, size_t S>
    // struct Signal
    // {
    //     T data[S];
    // };

    // inline static constexpr char ID[] = "example";
    // Signal<float, ID, 3> sig;

    /**
     * Constructor
     * @param drivers Pointer to the global drivers instance
     */
    RttTelemetry(tap::Drivers* drivers);

    /**
     * Set optional logging dependencies (can be called after construction)
     * @param controlInterface Control operator interface for input logging
     * @param refSerial Referee serial interface for game data logging
     * @param visionProcessor Vision coprocessor for CV data logging
     */
    void setLoggingDependencies(
        aruwsrc::control::ControlOperatorInterface* controlInterface = nullptr,
        tap::communication::serial::RefSerial* refSerial = nullptr,
        aruwsrc::serial::VisionCoprocessor* visionProcessor = nullptr);

    /**
     * Initialize the RTT telemetry system
     */
    void initialize();

    /**
     * Asynchronous telemetry update using modm protothreads.
     * Call this function repeatedly in your main loop.
     *
     * @return false when protothread completes (which never happens as this runs in infinite loop)
     */
    bool updateTelemetryAsync();

    /**
     * Log control operator interface data (joystick inputs, button states)
     */
    void logControlOperatorData();

    /**
     * Log referee system data (robot health, ammo, game state)
     */
    void logRefereeData();

    /**
     * Log vision coprocessor data (target detection, aim assist)
     */
    void logVisionData();

    /**
     * Log odometry state vector (position, velocity, orientation)
     * @param position Robot position in world frame (x, y in meters)
     * @param velocity Robot velocity in world frame (vx, vy in m/s)
     * @param orientation Robot orientation in radians
     */
    void logOdometryState(
        const modm::Vector2f& position,
        const modm::Vector2f& velocity,
        float orientation);

    template <typename T, const char* L, size_t S>
    void logSignal(const T (&data)[S])
    {
        emit_array_json_and_queue<T, L, S>(data);
    }

    template <typename T, const char* L>
    void logSignal(const T& value)
    {
        emit_scalar_json_and_queue<T, L>(value);
    }

private:
    tap::Drivers* drivers;

    // Optional logging dependencies (set via setLoggingDependencies)
    aruwsrc::control::ControlOperatorInterface* controlInterface;
    tap::communication::serial::RefSerial* refSerial;
    aruwsrc::serial::VisionCoprocessor* visionProcessor;

    // Timer for periodic telemetry updates
    tap::arch::PeriodicMilliTimer periodicTimer;

    // Timer for LED blinking
    tap::arch::PeriodicMilliTimer ledBlinkTimer;

    // Timer for extended logging data
    tap::arch::PeriodicMilliTimer extendedLoggingTimer;

    // Counter for periodic messages
    uint32_t messageCounter;

    // Flag to track if we've received first RTT input
    bool firstInputReceived;

    // Protothread state management
    enum class TelemetryState
    {
        IDLE,
        SENDING_HEARTBEAT,
        SENDING_EXTENDED_LOGGING,
        PROCESSING_INPUT
    };

    TelemetryState currentState;

    // Message queue for asynchronous transmission
    static constexpr size_t MAX_QUEUED_MESSAGES = 10;
    static constexpr size_t MAX_MESSAGE_SIZE = 512;

    struct QueuedMessage
    {
        char data[MAX_MESSAGE_SIZE];
        size_t length;
        bool valid;
    };

    QueuedMessage messageQueue[MAX_QUEUED_MESSAGES];
    size_t queueHead;
    size_t queueTail;
    size_t queueCount;

    /**
     * Queue a message for asynchronous transmission
     */
    void queueMessage(const char* message);

    /**
     * Send all queued messages (called by protothread)
     */
    void sendQueuedMessages();

    /**
     * Generate heartbeat message and queue it
     */
    void generateHeartbeatMessage();

    /**
     * Get current system timestamp in milliseconds
     * @return Timestamp in milliseconds since startup
     */
    uint32_t getTimestamp() const;

    template <class T>
    void append_json_value(std::string& out, const T& v)
    {
        if constexpr (std::is_floating_point_v<T>)
        {
            char buf[64];
            // compact but precise enough; adjust if you need fixed decimals
            int vq = static_cast<int32_t>(v * 1000);
            std::snprintf(buf, sizeof(buf), "%d.%03d", vq / 1000, vq % 1000);
            out += buf;
        }
        else if constexpr (std::is_integral_v<T> && !std::is_same_v<T, bool>)
        {
            char buf[64];
            // print as signed long long to be safe
            std::snprintf(buf, sizeof(buf), "%lld", static_cast<long long>(v));
            out += buf;
        }
        else if constexpr (std::is_same_v<T, bool>)
        {
            out += (v ? "true" : "false");
        }
        else if constexpr (std::is_same_v<T, const char*> || std::is_same_v<T, char*>)
        {
            // minimal string escaping (quotes/backslashes); extend if needed
            out += '"';
            for (const char* p = v; *p; ++p)
            {
                char c = *p;
                if (c == '"' || c == '\\') out += '\\';
                out += c;
            }
            out += '"';
        }
        else if constexpr (std::is_same_v<T, char>)
        {
            // represent char as a small JSON string
            out += '"';
            if (v == '"' || v == '\\') out += '\\';
            out += v;
            out += '"';
        }
        else
        {
            static_assert(std::is_arithmetic_v<T>, "Unsupported type for JSON logging");
        }
    }

    template <class T, const char* L>
    void emit_scalar_json_and_queue(const T& v)
    {
        std::string msg;
        msg.reserve(64);
        msg += "{\"";
        msg += L;
        msg += "\":";
        append_json_value(msg, v);
        msg += '}';

        // queue: assumes queueMessage copies the string
        queueMessage(msg.c_str());
    }

    template <class T, const char* L, size_t S>
    void emit_array_json_and_queue(const T (&arr)[S])
    {
        std::string msg;
        msg.reserve(32 + S * 16);  // rough reserve

        msg += "{\"";
        msg += L;
        msg += "\":[";

        for (size_t i = 0; i < S; ++i)
        {
            if (i) msg += ',';
            append_json_value(msg, arr[i]);
        }
        msg += "]}\n";

        queueMessage(msg.c_str());
    }
};

// template <typename T, const char* L>
// void logSignal(T data)
// {
//     //
// }

}  // namespace aruwsrc::communication::serial

#endif  // RTT_TELEMETRY_HPP_