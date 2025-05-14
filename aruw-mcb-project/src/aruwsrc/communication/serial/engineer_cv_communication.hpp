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
#include "tap/drivers.hpp"
#include "tap/algorithms/transforms/transform.hpp"

namespace aruwsrc
{
namespace serial
{
class EngineerCVCommunication : public tap::communication::serial::DJISerial
{
public:
    static constexpr tap::communication::serial::Uart::UartPort ENGINEER_CV_RX_UART_PORT =
        tap::communication::serial::Uart::UartPort::Uart8;
    static constexpr size_t ENGINEER_CV_UART_BAUD_RATE = 115'200;

    EngineerCVCommunication(tap::Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(EngineerCVCommunication);
    mockable ~EngineerCVCommunication();

    struct PositionData
    {
        float xPos;  ///< x position of the target (in cm).
        float yPos;  ///< y position of the target (in cm).
        float zPos;  ///< z position of the target (in cm).
    } modm_packed;

    struct RotationData
    {
        float alpha;
        float beta;
        float gamma;
    } modm_packed;

    struct TargetPositionMessage
    {
        PositionData posData;
        RotationData rotData;
    } modm_packed;

    /**
     * Handles the types of messages defined above in the RX message handlers section.
     */
    void messageReceiveCallback(const ReceivedSerialMessage& completeMessage) override;

    /**
     * Call this before using the serial line, initializes the uart line
     * and the callback
     */
    mockable void initializeCV();

    inline const tap::algorithms::transforms::Transform& getReceptableToCam() const
    {
        return receptableToCam;
    }

    // @todo private should not be here
private:
    static EngineerCVCommunication* engineerCVCommunicationInstance;
    TargetPositionMessage targetPositionMessage;
    tap::algorithms::transforms::Transform receptableToCam;
};
}  // namespace serial
}  // namespace aruwsrc

#endif  // VISION_COPROCESSOR_HPP_
