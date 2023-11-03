/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SERIAL_MCB_LITE_HPP_
#define SERIAL_MCB_LITE_HPP_

#include "tap/communication/can/can_bus.hpp"
#include "tap/communication/serial/dji_serial.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

#include "modm/container/queue.hpp"
#include "motor/virtual_can_rx_handler.hpp"
#include "motor/virtual_dji_motor_tx_handler.hpp"
#include "sensors-interface/virtual_analog.hpp"
#include "sensors-interface/virtual_current_sensor.hpp"
#include "sensors-interface/virtual_digital.hpp"
#include "sensors-interface/virtual_imu_interface.hpp"
#include "sensors-interface/virtual_leds.hpp"
#include "sensors-interface/virtual_pwm.hpp"

#include "message_types.hpp"

using namespace tap::communication::sensors::imu::mpu6500;

namespace aruwsrc::virtualMCB
{
/**
 * This class is used to communicate with the the virtual MCB using the UART port.
 * This class handles the sending and receiving of motor data, as well as receiving
 * IMU and current sensor data. To use: call initialize() once to setup the port baudrate.
 * Afterwards, call updateSerial() as fast as possible to process receiving data as fast as
 * possible. Call sendData per loop to send motor desired outputs at a consistent rate.
 */
class SerialMCBLite : public tap::communication::serial::DJISerial
{
public:
    SerialMCBLite(tap::Drivers* drivers, tap::communication::serial::Uart::UartPort port);

    void messageReceiveCallback(const ReceivedSerialMessage& completeMessage) override;

    void sendData();

    void initialize();

    constexpr static int UART_BAUDRATE = 500'000;

    VirtualCanRxHandler canRxHandler;
    VirtualDJIMotorTxHandler motorTxHandler;
    VirtualCurrentSensor currentSensor;
    VirtualIMUInterface imu;
    VirtualDigital digital;
    VirtualAnalog analog;
    VirtualPwm pwm;
    VirtualLeds leds;

    // Debug method
    void updateEverything()
    {
        if (updateIO)
        {
            imu.requestCalibration();

            digital.set(tap::gpio::Digital::OutputPin::E, true);
            digital.configureInputPullMode(
                tap::gpio::Digital::InputPin::B,
                tap::gpio::Digital::InputPullMode::PullUp);

            pwm.write(0.5f, tap::gpio::Pwm::Buzzer);
            pwm.setTimerFrequency(tap::gpio::Pwm::TIMER12, 440);
            pwm.start(tap::gpio::Pwm::TIMER12);

            leds.set(tap::gpio::Leds::LedPin::B, true);
        }
    };

private:
    void processCanMessage(const ReceivedSerialMessage& completeMessage, tap::can::CanBus canbus);

    void processIMUMessage(const ReceivedSerialMessage& completeMessage);

    void processCurrentSensorMessage(const ReceivedSerialMessage& completeMessage);

    void processAnalogMessage(const ReceivedSerialMessage& completeMessage);

    void processDigitalMessage(const ReceivedSerialMessage& completeMessage);

    tap::communication::serial::Uart::UartPort port;

    // DEBUG VARIABLES
    IMUMessage currentIMUData;
    CurrentSensorInputMessage currentCurrentSensorData;
    AnalogInputPinMessage currentAnalogData;
    DigitalInputPinMessage currentDigitalData;
    uint8_t can1Data[64];
    uint8_t can2Data[64];

qwjwpowqejpqw

    uint8_t gotAFullMessage;           
    uint8_t gotAIMUMessage;              

    bool updateIO = false;

};
}  // namespace aruwsrc::virtualMCB      
#endif
  