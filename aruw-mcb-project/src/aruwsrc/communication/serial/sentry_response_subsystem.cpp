/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "sentry_response_subsystem.hpp"

#include "aruwsrc/drivers.hpp"

#include "sentry_request_message_types.hpp"

namespace aruwsrc::communication::serial
{
SentryResponseSubsystem::SentryResponseSubsystem(
    aruwsrc::Drivers &drivers,
    aruwsrc::control::sentry::drive::SentryAutoDriveComprisedCommand &driveCommand)
    : tap::control::Subsystem(&drivers),
      drivers(drivers),
      driveCommand(driveCommand),
      refSerialTransmitter(&drivers)
{
}

void SentryResponseSubsystem::refresh() { this->run(); }

bool SentryResponseSubsystem::run()
{
    PT_BEGIN();

    PT_WAIT_UNTIL(drivers.refSerial.getRefSerialReceivingData());

    while (true)
    {
        if (this->sentryMoving != this->getDriveStatus())
        {
            this->sentryMoving = this->getDriveStatus();

            this->robotToRobotMessage.dataAndCRC16[0] = static_cast<uint8_t>(this->sentryMoving);

            PT_CALL(refSerialTransmitter.sendRobotToRobotMsg(
                &this->robotToRobotMessage,
                SENTRY_RESPONSE_MESSAGE_ID,
                drivers.refSerial.getRobotIdBasedOnCurrentRobotTeam(
                    tap::communication::serial::RefSerialData::RobotId::BLUE_HERO),
                1));

            PT_CALL(this->refSerialTransmitter.sendRobotToRobotMsg(
                &this->robotToRobotMessage,
                SENTRY_RESPONSE_MESSAGE_ID,
                drivers.refSerial.getRobotIdBasedOnCurrentRobotTeam(
                    tap::communication::serial::RefSerialData::RobotId::BLUE_SOLDIER_1),
                1));
        }

        PT_YIELD();
    }

    PT_END();
}

bool SentryResponseSubsystem::getDriveStatus()
{
    if (!this->drivers.commandScheduler.isCommandScheduled(&this->driveCommand))
    {
        return false;
    }

    return this->driveCommand.getMovementStatus();
}
}  // namespace aruwsrc::communication::serial
