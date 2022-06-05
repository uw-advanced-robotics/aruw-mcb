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

#ifndef SENTINEL_DRIVE_STATUS_HUD_INDICATOR_HPP_
#define SENTINEL_DRIVE_STATUS_HUD_INDICATOR_HPP_

#include "tap/architecture/periodic_timer.hpp"
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

#include "aruwsrc/control/client-display/hud_indicator.hpp"
#include "aruwsrc/control/sentinel/drive/sentinel_auto_drive_comprised_command.hpp"
#include "modm/processing/resumable.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::client_display
{
class SentinelDriveStatusHudIndicator : public aruwsrc::control::client_display::HudIndicator,
                                        protected modm::Resumable<2>
{
public:
    static constexpr uint16_t SENTINEL_DRIVE_STATUS_Y = 660;
    static constexpr Tx::GraphicColor SENTINEL_DRIVE_MOVING = Tx::GraphicColor::GREEN;
    static constexpr Tx::GraphicColor SENTINEL_DRIVE_NOT_MOVING = Tx::GraphicColor::PURPLISH_RED;

    enum class RobotHudsToUpdate
    {
        SOLDIER_3 = 0,
        HERO,
        NUM_ROBOT_HUDS,
    };

    static constexpr RobotId ROBOT_HUD_TO_UPDATE_TO_ROBOT_ID_MAP[] = {
        RobotId::RED_SOLDIER_1,
        RobotId::RED_HERO,
    };

    SentinelDriveStatusHudIndicator(
        aruwsrc::Drivers &drivers,
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
        aruwsrc::control::sentinel::drive::SentinelAutoDriveComprisedCommand &command);

    modm::ResumableResult<bool> sendInitialGraphics() override;

    modm::ResumableResult<bool> update() override;

    void initialize() override;

private:
    aruwsrc::Drivers &drivers;

    const aruwsrc::control::sentinel::drive::SentinelAutoDriveComprisedCommand &command;

    Tx::GraphicCharacterMessage indicatorText[size_t(RobotHudsToUpdate::NUM_ROBOT_HUDS)];
    Tx::Graphic1Message indicatorCircleOutline[size_t(RobotHudsToUpdate::NUM_ROBOT_HUDS)];
    Tx::Graphic1Message indicatorCircle[size_t(RobotHudsToUpdate::NUM_ROBOT_HUDS)];

    size_t i = 0;

    bool prevChassisMovementStatus = false;

    tap::arch::PeriodicMilliTimer resendBaseGraphicDataTimer{10'000};

    bool run();

    bool getChassisMovementStatus() const;

    void configFrameIdForSpecificRobot(Tx::Graphic1Message *message, uint16_t clientRobotId);

    void configFrameIdForSpecificRobot(
        Tx::GraphicCharacterMessage *message,
        uint16_t clientRobotId);
};
}  // namespace aruwsrc::control::client_display

#endif  // SENTINEL_DRIVE_STATUS_HUD_INDICATOR_HPP_
