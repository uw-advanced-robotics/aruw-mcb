/*
 * Copyright (c) 2021-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

 #ifndef ENGINEER_CV_COMMUNICATION_HPP_
 #define ENGINEER_CV_COMMUNICATION_HPP_

 #include "tap/communication/serial/dji_serial.hpp"

 namespace aruwsrc
 {
 namespace serial
 {

 class EngineerCVCommunication : public tap::communication::serial::DJISerial
 {
 public:
     static constexpr tap::communication::serial::Uart::UartPort VISION_COPROCESSOR_TX_UART_PORT =
         tap::communication::serial::Uart::UartPort::Uart2;
 
     static constexpr tap::communication::serial::Uart::UartPort VISION_COPROCESSOR_RX_UART_PORT =
         tap::communication::serial::Uart::UartPort::Uart3;
 
 
    EngineerCVCommunication(tap::Drivers* drivers);
     DISALLOW_COPY_AND_ASSIGN(EngineerCVCommunication);
     mockable ~EngineerCVCommunication();
 
 
     /**
      * Handles the types of messages defined above in the RX message handlers section.
      */
     void messageReceiveCallback(const ReceivedSerialMessage& completeMessage) override;
 
     // @todo private should not be here
 private:

    static EngineerCVCommunication* engineerCVCommunicationInstance;
 };
 }  // namespace serial
 }  // namespace aruwsrc
 
 #endif  // VISION_COPROCESSOR_HPP_
 