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

#ifndef ARUW_VOLTAGE_CURRENT_SENSOR_HPP_
#define ARUW_VOLTAGE_CURRENT_SENSOR_HPP_

#include "tap/architecture/timeout.hpp"
#include "tap/communication/can/can_rx_listener.hpp"
#include "tap/communication/sensors/current/current_sensor_interface.hpp"
#include "tap/communication/sensors/voltage/voltage_sensor_interface.hpp"
#include "tap/drivers.hpp"

#include "modm/architecture/interface/can_message.hpp"

namespace aruwsrc::can
{
static constexpr uint16_t CHASSIS_SENSOR_CAN_ID = 0x1C5;

class AruwVoltageCurrentSensor
    : public tap::can::CanRxListener,
      public tap::communication::sensors::voltage::VoltageSensorInterface,
      public tap::communication::sensors::current::CurrentSensorInterface
{
public:
    AruwVoltageCurrentSensor(tap::Drivers* drivers, tap::can::CanBus canBus);

    void processMessage(const modm::can::Message& message) override;

    mockable void initialize();

    float getVoltageMv() const override { return this->voltage; };
    float getCurrentMa() const override { return this->current; };

    void update() override{};

    bool isOnline() const { return !this->heartbeat.isExpired(); }

private:
    float voltage = 0;
    float current = 0;

    tap::arch::MilliTimeout heartbeat;
};
}  // namespace aruwsrc::can

#endif  // ARUW_VOLTAGE_CURRENT_SENSOR_HPP_
