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
#ifndef TARGET_SHARE_TRANSMITTER_SUBSYSTEM_HPP_
#define TARGET_SHARE_TRANSMITTER_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

#include "inter_robot_message_transmitter.hpp"
#include "robot_share_message_types.hpp"

namespace aruwsrc::communication::serial
{

class RobotShareTransmitterSubsystem : public tap::control::Subsystem
{
public:
    RobotShareTransmitterSubsystem(
        tap::Drivers* drivers,
        tap::communication::serial::RefSerialData::RobotId recipient);

    void initialize() override;

    void refresh() override;

    void refreshSafeDisconnect() override;

    const char* getName() const override { return "Robot-share transmitter subsystem"; }

    inline mockable void queueRequest(RobotShareMessageType type)
    {
        robotShareTransmitter.queueMessage(type);
    }

private:
    InterRobotMessageTransmitter<
        RobotShareMessageType,
        static_cast<uint8_t>(RobotShareMessageType::NUM_MESSAGE_TYPES)>
        robotShareTransmitter;
};  // class RobotShareTransmitterSubsystem

}  // namespace aruwsrc::communication::serial
#endif  // TARGET_SHARE_TRANSMITTER_SUBSYSTEM_HPP_
