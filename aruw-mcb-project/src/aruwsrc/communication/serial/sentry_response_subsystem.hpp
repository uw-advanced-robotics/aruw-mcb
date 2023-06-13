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

#ifndef SENTRY_RESPONSE_SUBSYSTEM_HPP_
#define SENTRY_RESPONSE_SUBSYSTEM_HPP_

#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/subsystem.hpp"

#include "aruwsrc/control/chassis/sentry/sentry_auto_drive_comprised_command.hpp"
#include "modm/processing/protothread.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::communication::serial
{
/**
 * Subsystem that handles responding to sentry requests. Currently, this subsystem only handles
 * sending the current drive status of the sentry. In other words, the sentry will send to other
 * robots whether or not it is automatically moving or not.
 */
class SentryResponseSubsystem : public tap::control::Subsystem, ::modm::pt::Protothread
{
public:
    /**
     * @param[in] drivers Reference to a global drivers instance.
     * @param[in] driveCommand A reference to a global `SentryAutoDriveComprisedCommand`. Used to
     * check if the robot is driving or not.
     */
    SentryResponseSubsystem(
        tap::Drivers &drivers,
        aruwsrc::control::sentry::drive::SentryAutoDriveComprisedCommand &driveCommand);

    void refresh() override;

    void refreshSafeDisconnect() override { stop(); }

private:
    tap::Drivers &drivers;
    aruwsrc::control::sentry::drive::SentryAutoDriveComprisedCommand &driveCommand;

    tap::communication::serial::RefSerialTransmitter refSerialTransmitter;

    /// Message to be sent by the sentry to other robots.
    tap::communication::serial::RefSerialData::Tx::RobotToRobotMessage robotToRobotMessage;

    bool sentryMoving = true;

    bool run();

    /// @return True if the `driveCommand` is scheduled and the command is in the moving state.
    bool getDriveStatus();
};
}  // namespace aruwsrc::communication::serial

#endif  //  SENTRY_RESPONSE_SUBSYSTEM_HPP_
