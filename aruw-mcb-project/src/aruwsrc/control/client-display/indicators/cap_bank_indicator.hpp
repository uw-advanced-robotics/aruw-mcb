/*
 * Copyright (c) 2020-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef CAP_BANK_INDICATOR_HPP_
#define CAP_BANK_INDICATOR_HPP_

#include "tap/architecture/periodic_timer.hpp"
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

#include "aruwsrc/communication/can/cap-bank/capacitor_bank.hpp"
#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

namespace tap
{
class Drivers;
}

namespace aruwsrc::control::client_display::indicators
{
/**
 * Draws a bar on the HUD that represents the current charge of the cap bank.
 */
class CapBankIndicator : public HudIndicator, protected modm::Resumable<2>
{
public:
    /**
     * Construct a CapBankIndicator.
     *
     * @param[in] refSerialTransmitter Transmitter that stores ref serial transmission state for the
     * protothread that this indicator is used in.
     * @param[in] capBank A pointer to the capacitor bank for the robot.
     */
    CapBankIndicator(
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
        const communication::can::cap_bank::CapacitorBank *capBank);

    modm::ResumableResult<void> sendInitialGraphics() override final;

    modm::ResumableResult<void> update() override final;

    void initialize() override final;

private:
    // X position of the text
    static constexpr uint16_t TEXT_X = SCREEN_WIDTH / 2 + 50;
    static constexpr uint16_t NUMBER_X = TEXT_X + 330;
    // Y position of the text
    static constexpr uint16_t TEXT_Y = 900;
    // SIZE of the text.
    static constexpr uint16_t SIZE = 40;
    // WIDTH of the text
    static constexpr uint16_t WIDTH = 4;

    static constexpr uint16_t VOLTAGE_SQUARED_MAX = pow(30, 2);
    // cap voltage below which the number turns from green to yellow
    static constexpr uint16_t VOLTAGE_SQUARED_YELLOW = pow(20, 2);
    // cap voltage below which the number turns from yellow to orange
    static constexpr uint16_t VOLTAGE_SQUARED_ORANGE = pow(15, 2);
    // minimum displayed voltage of the supercapacitors (minimum usable voltage)
    static constexpr uint16_t VOLTAGE_SQUARED_MIN =
        pow(communication::can::cap_bank::CAPACITOR_BANK_MIN_VOLTAGE, 2);

    const communication::can::cap_bank::CapacitorBank *capBank;

    /**
     * A number that shows the charge % of the Capacitor Bank.
     */
    Tx::Graphic1Message numberGraphic;
    Tx::Graphic1Message backgroundGraphic;
    tap::communication::referee::StateHUDIndicator<int32_t> numberIndicator;

    Tx::GraphicColor previousColor;

    /**
     * A graphic that represents the current status of the Capacitor Bank.
     */
    Tx::GraphicCharacterMessage capBankTextGraphic;

    communication::can::cap_bank::State previousState;

    static inline void updateVoltage(int32_t value, RefSerialData::Tx::Graphic1Message *graphic)
    {
        tap::communication::serial::RefSerialTransmitter::configInteger(
            SIZE,
            WIDTH,
            NUMBER_X,
            TEXT_Y,
            value,
            &graphic->graphicData);
    }
};
}  // namespace aruwsrc::control::client_display::indicators

#endif  //  CAP_BANK_INDICATOR_HPP_
