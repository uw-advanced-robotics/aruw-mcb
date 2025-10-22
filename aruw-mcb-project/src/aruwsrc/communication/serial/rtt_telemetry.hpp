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

#include "tap/architecture/periodic_timer.hpp"

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
 * This class provides a non-blocking interface for streaming telemetry data
 * through the J-Link RTT channel 0, which can be accessed via OpenOCD and
 * telnet on port 9090.
 */
class RttTelemetry
{
public:
    /**
     * Constructor
     * @param drivers Pointer to the global drivers instance
     */
    RttTelemetry(tap::Drivers* drivers);

    /**
     * Initialize the RTT telemetry system
     */
    void initialize();

    /**
     * Send about menu information as telemetry
     * @param robotName The robot target name
     * @param lastUser The username who built the code
     * @param lastSha The git SHA of the build
     * @param lastDate The build date
     * @param branchName The git branch name
     */
    void sendAboutInfo(
        const char* robotName,
        const char* lastUser,
        const char* lastSha,
        const char* lastDate,
        const char* branchName);

    /**
     * Send a formatted log message with timestamp
     * @param level Log level (INFO, DEBUG, WARNING, ERROR)
     * @param message The message to send
     */
    void sendLogMessage(const char* level, const char* message);

    /**
     * Check if RTT is ready to send more data
     * @return true if transmit buffer has space available
     */
    bool isReady() const;

    /**
     * Update method to be called periodically to send periodic telemetry
     */
    void update();

private:
    tap::Drivers* drivers;

    // Timer for periodic telemetry updates
    tap::arch::PeriodicMilliTimer periodicTimer;

    // Counter for periodic messages
    uint32_t messageCounter;

    /**
     * Get current system timestamp in milliseconds
     * @return Timestamp in milliseconds since startup
     */
    uint32_t getTimestamp() const;

    /**
     * Send a formatted message with JSON-like structure
     */
    void sendFormattedMessage(const char* type, const char* data);
};

}  // namespace aruwsrc::communication::serial

#endif  // RTT_TELEMETRY_HPP_