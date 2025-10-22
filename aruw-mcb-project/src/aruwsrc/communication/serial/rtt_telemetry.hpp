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
 * Provides simple heartbeat functionality and basic input detection for future
 * logging system expansion.
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
     * Update method to be called periodically to send periodic telemetry
     * and check for incoming messages
     */
    void update();

private:
    tap::Drivers* drivers;

    // Timer for periodic telemetry updates
    tap::arch::PeriodicMilliTimer periodicTimer;
    
    // Timer for LED blinking
    tap::arch::PeriodicMilliTimer ledBlinkTimer;

    // Counter for periodic messages
    uint32_t messageCounter;
    
    // Flag to track if we've received first RTT input
    bool firstInputReceived;

    /**
     * Get current system timestamp in milliseconds
     * @return Timestamp in milliseconds since startup
     */
    uint32_t getTimestamp() const;
};

}  // namespace aruwsrc::communication::serial

#endif  // RTT_TELEMETRY_HPP_