/*
 * Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef IK_SERIAL_HPP_
#define IK_SERIAL_HPP_

#include "tap/communication/serial/dji_serial.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/robot/engineer/arm/arm_superstructure.hpp"

namespace aruwsrc::engineer
{
struct IKSolution
{
    float liftSetpoint;
    float xAxisSetpoint;
    float yawSetpoint;
    float pitchSetpoint;
    float rollSetpoint;
    bool solutionFound;
} modm_packed;

struct SlotDelta
{
    float x;
    float y;
    float z;
    float yaw;
    float pitch;
    float roll;
} modm_packed;

class IKSerial : public tap::communication::serial::DJISerial
{
public:
    IKSerial(tap::Drivers *drivers, tap::communication::serial::Uart::UartPort port)
        : DJISerial(drivers, port),
          port(port)
    {
        positionMessage.messageType = JOINT_POSITION_MESSAGE_ID;
    }

    void messageReceiveCallback(const ReceivedSerialMessage &completeMessage) override;

    void sendJointPositions();

    IKSolution getIKSolution() const { return ikSolution; }

    SlotDelta getSlotDelta() const { return slotDelta; }

    void attachSuperstructure(aruwsrc::engineer::arm::ArmSuperstructure *superstructure)
    {
        this->superstructure = superstructure;
    }

    static constexpr uint8_t IK_SOLUTION_MESSAGE_ID = 0x01;
    static constexpr uint8_t SLOT_DELTA_MESSAGE_ID = 0x02;
    static constexpr uint8_t JOINT_POSITION_MESSAGE_ID = 0x03;

private:
    tap::communication::serial::Uart::UartPort port;
    IKSolution ikSolution;
    SlotDelta slotDelta;
    aruwsrc::engineer::arm::ArmSuperstructure *superstructure;

    DJISerial::SerialMessage<sizeof(aruwsrc::engineer::arm::Position)> positionMessage;

};
}  // namespace aruwsrc::engineer
#endif  // IK_SERIAL_HPP_
