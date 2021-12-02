/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef CLIENT_DISPLAY_COMMAND_HPP_
#define CLIENT_DISPLAY_COMMAND_HPP_

#include "tap/architecture/periodic_timer.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/control/command.hpp"

#include "modm/processing/protothread.hpp"
#include "modm/processing/resumable.hpp"

namespace tap::control
{
class Subsystem;
}

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::display
{
class ClientDisplaySubsystem;

class ClientDisplayCommand : public tap::control::Command,
                             ::modm::pt::Protothread,
                             modm::Resumable<4>
{
public:
    ClientDisplayCommand(
        aruwsrc::Drivers *drivers,
        ClientDisplaySubsystem *clientDisplay,
        const tap::control::Command *wiggleCommand,
        const tap::control::Command *followTurret,
        const tap::control::Command *beybladeCommand,
        const tap::control::Command *baseDriveCommand);

    const char *getName() const override { return "client display"; }

    void initialize() override {}

    void execute() override;

    void end(bool) override {}

    bool isFinished() const override { return false; }

private:
    static constexpr uint16_t SCREEN_WIDTH = 1920;
    static constexpr uint16_t SCREEN_HEIGHT = 1080;
    static constexpr uint16_t FONT_SIZE = 30;
    static constexpr uint16_t FONT_THICKNESS = 4;
    static constexpr uint16_t LINE_THICKNESS = 4;
    static constexpr uint16_t TEXT_TOP_ROW_Y = 850;
    static constexpr uint16_t SCREEN_MARGIN = 100;
    static constexpr uint8_t DRIVE_COMMAND_GRAPHIC_LAYER = 1;
    static constexpr uint8_t RETICLE_GRAPHIC_LAYER = 2;
    static constexpr uint8_t CAP_BANK_LAYER_1 = 3;
    static constexpr uint8_t CAP_BANK_LAYER_2 = 4;
    static constexpr int32_t DELAY_PERIOD_BTWN_SENDS = 110;
    static constexpr uint8_t RETICLE_LINE1_NAME[] = {0, 0, 0};
    static constexpr uint8_t RETICLE_LINE2_NAME[] = {0, 0, 1};
    static constexpr uint8_t RETICLE_LINE3_NAME[] = {0, 0, 2};
    static constexpr uint8_t RETICLE_LINE4_NAME[] = {0, 0, 3};
    static constexpr uint8_t RETICLE_CIRCLE_NAME[] = {0, 0, 4};
    static constexpr uint8_t DRIVE_TEXT_NAME[] = {0, 0, 5};
    static constexpr uint8_t CAP_TEXT_NAME[] = {0, 0, 6};
    static constexpr uint8_t CAP_VALUE_NAME[] = {0, 0, 7};

    aruwsrc::Drivers *drivers;

    // General variables
    /// @note The maximum frequency of this timer is 10 Hz according to RM rules.
    tap::arch::MilliTimeout delayTimer{DELAY_PERIOD_BTWN_SENDS};

    // Drive related variables
    const tap::control::Command *wiggleCommand;
    const tap::control::Command *followTurretCommand;
    const tap::control::Command *beybladeCommand;
    const tap::control::Command *baseDriveCommand;
    const tap::control::Command *currDriveCommandScheduled = nullptr;
    const tap::control::Command *newDriveCommandScheduled = nullptr;
    tap::serial::RefSerial::Tx::GraphicCharacterMessage driveCommandMsg;
    tap::serial::RefSerial::Tx::GraphicColor driveCommandColor;
    tap::arch::PeriodicMilliTimer addDriveCommandTimer{10000};

    // Turret reticle variables
    static constexpr uint16_t TURRET_RETICLE_1M_WIDTH = 150;
    static constexpr uint16_t TURRET_RETICLE_3M_WIDTH = 100;
    static constexpr uint16_t TURRET_RETICLE_5M_WIDTH = 50;
#ifdef TARGET_HERO  // TODO tune the things
    static constexpr uint16_t TURRET_RETICLE_1MY = 500;
    static constexpr uint16_t TURRET_RETICLE_3MY = 250;
    static constexpr uint16_t TURRET_RETICLE_5MY = 200;
#else
    static constexpr uint16_t TURRET_RETICLE_1MY = 500;
    static constexpr uint16_t TURRET_RETICLE_3MY = 400;
    static constexpr uint16_t TURRET_RETICLE_5MY = 300;
#endif
    tap::arch::PeriodicMilliTimer sendReticleTimer{10000};
    tap::serial::RefSerial::Tx::Graphic5Message reticleMsg;

    // Cap bank related variables
    tap::arch::PeriodicMilliTimer sendCapBankTimer{10000};
    tap::serial::RefSerial::Tx::GraphicCharacterMessage capStringMsg;
    tap::serial::RefSerial::Tx::Graphic1Message capPowerRemainMsg;
    int capMsgAdded = 0;
    int32_t capicatance = 0;

    modm::ResumableResult<bool> initializeNonblocking();
    bool run();

    void initDriveCommandMsg();
    modm::ResumableResult<bool> updateDriveCommandMsg();

    void initTurretReticleMsg();
    modm::ResumableResult<bool> updateTurretReticleMsg();

    void initCapBankMsg();
    modm::ResumableResult<bool> updateCapBankMsg();
};
}  // namespace aruwsrc::display

#endif  // CLIENT_DISPLAY_COMMAND_HPP_
