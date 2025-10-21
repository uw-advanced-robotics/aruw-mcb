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

#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"

namespace aruwsrc::communication::serial
{
RttTelemetry::RttTelemetry(tap::Drivers* drivers)
    : drivers(drivers),
      rtt(0),  // Use RTT channel 0
      rttDevice(rtt),
      rttStream(rttDevice),
      periodicTimer(1000),  // 1 second periodic updates
      messageCounter(0)
{
}

void RttTelemetry::initialize()
{
    // RTT is automatically initialized by the modm framework
    // Send initialization message
    rttStream << "{\"type\":\"init\",\"timestamp\":" << getTimestamp()
              << ",\"message\":\"RTT Telemetry Initialized\"}" << modm::endl;
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
    // Cast away const since transmitBufferSize() is not const but doesn't modify state
    return const_cast<modm::platform::Rtt&>(rtt).transmitBufferSize() >
           100;  // Keep some buffer space
}

void RttTelemetry::update()
{
    if (periodicTimer.execute())
    {
        // Send periodic heartbeat with system information
        rttStream << "{\"type\":\"heartbeat\",\"timestamp\":" << getTimestamp()
                  << ",\"counter\":" << messageCounter++ << ",\"data\":{"
                  << "\"uptime\":" << tap::arch::clock::getTimeMilliseconds() << ","
                  << "\"bufferSize\":" << rtt.transmitBufferSize() << "}}" << modm::endl;
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