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

#ifndef CAP_BANK_TEXT_INDICATOR_HPP_
#define CAP_BANK_TEXT_INDICATOR_HPP_

#include "tap/architecture/periodic_timer.hpp"
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

#include "aruwsrc/communication/can/capacitor_bank.hpp"
#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

namespace tap
{
class Drivers;
}

namespace aruwsrc::control::client_display
{
/**
 * Draws a bar on the HUD that represents the current charge of the cap bank.
 */
class CapBankTextIndicator : public HudIndicator, protected modm::Resumable<2>
{
public:
    /**
     * Construct a CapBankIndicator.
     *
     * @param[in] refSerialTransmitter Transmitter that stores ref serial transmission state for the
     * protothread that this indicator is used in.
     * @param[in] capBank A pointer to the capacitor bank for the robot.
     */
    CapBankTextIndicator(
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
        const can::capbank::CapacitorBank *capBank);

    modm::ResumableResult<void> sendInitialGraphics() override final;

    modm::ResumableResult<void> update() override final;

    void initialize() override final;

private:
    // X position of the text
    static constexpr uint16_t TEXT_X = SCREEN_WIDTH / 2 - 150;
    // Y position of the text
    static constexpr uint16_t TEXT_Y = 100;
    // WIDTH of the text
    static constexpr uint16_t WIDTH = 4;
    // SIZE of the text
    static constexpr uint16_t SIZE = 40;
    static constexpr uint16_t NUMBER_X = TEXT_X + 175;

    // Indicator bar is in units of voltage squared so it is proportional to energy,
    //   but doesnt depend on differences in capacitance between robots, and limits can be easily
    //   changed
    // maximum displayed voltage of the supercapacitors (full charge)
    static constexpr uint16_t VOLTAGE_SQUARED_MAX = pow(30, 2);
    // minimum displayed voltage of the supercapacitors (minimum usable voltage)
    static constexpr uint16_t VOLTAGE_SQUARED_MIN =
        pow(can::capbank::CAPACITOR_BANK_MIN_VOLTAGE, 2);

    const can::capbank::CapacitorBank *capBank;

    /**
     * A int that shows the charge of the Capacitor Bank 0 - 100%.
     */
    Tx::Graphic1Message capBankVoltageLevel;
    tap::arch::PeriodicMilliTimer voltageUpdateTimer;

    /**
     * A graphic that represents the current status of the Capacitor Bank.
     */
    Tx::GraphicCharacterMessage capBankTextGraphic;

    can::capbank::State previousState;

    int32_t voltage;
};
}  // namespace aruwsrc::control::client_display

#endif  //  CAP_BANK_TEXT_INDICATOR_HPP_
