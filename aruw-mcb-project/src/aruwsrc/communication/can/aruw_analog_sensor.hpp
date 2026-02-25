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

#ifndef ARUW_ANALOG_SENSOR_HPP_
#define ARUW_ANALOG_SENSOR_HPP_

#include "tap/architecture/timeout.hpp"
#include "tap/communication/can/can_rx_listener.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/communication/rtt/rtt_telemetry.hpp"
#include "modm/architecture/interface/can_message.hpp"

namespace aruwsrc::communication::can
{

class AruwAnalogSensor : public tap::can::CanRxListener
{
public:
    AruwAnalogSensor(tap::Drivers* drivers, tap::can::CanBus canBus, uint16_t canId = 0x1D6);

    void processMessage(const modm::can::Message& message) override;

    mockable void initialize();

    uint16_t getAnalogInput0() const { return this->ai0; }
    uint16_t getAnalogInput1() const { return this->ai1; }

    bool isOnline() const { return !this->heartbeat.isExpired(); }

    inline void sendRTTTelemetry(
        communication::rtt::RttTelemetry* rttTelemetry,
        const char* namePrefix = "sensor:analog_sensor") const
    {
        rttTelemetry->logSignal((std::string(namePrefix) + "_ai0").c_str(), this->ai0);
        rttTelemetry->logSignal((std::string(namePrefix) + "_ai1").c_str(), this->ai1);
    }

private:
    uint16_t ai0 = 0;
    uint16_t ai1 = 0;

    const uint16_t CAN_ID;
    tap::arch::MilliTimeout heartbeat;
};
}  // namespace aruwsrc::communication::can

#endif  // ARUW_ANALOG_SENSOR_HPP_
