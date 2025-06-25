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

#ifndef CAP_BANK_INDICATOR_HPP_
#define CAP_BANK_INDICATOR_HPP_

#include "tap/architecture/periodic_timer.hpp"
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
        const can::capbank::CapacitorBank *capBank);

    modm::ResumableResult<void> sendInitialGraphics() override final;

    modm::ResumableResult<void> update() override final;

    void initialize() override final;

private:
    /** The X location of the center of the cap bank bar on the screen, in pixels. */
    static constexpr uint16_t CAP_CENTER_X = 1870;
    /** The Y location of the center of the cap bank bar on the screen, in pixels. */
    static constexpr uint16_t CAP_CENTER_Y = 630;
    /** The height of the cap bank bar, in pixels. */
    static constexpr uint16_t BOX_HEIGHT = 300;
    /** The width of the cap bank bar, in pixels. */
    static constexpr uint16_t BOX_WIDTH = 50;

    // Indicator bar is in units of voltage squared so it is proportional to energy,
    //   but doesnt depend on differences in capacitance between robots, and limits can be easily
    //   changed
    // maximum displayed voltage of the supercapacitors (full charge)
    static constexpr uint16_t VOLTAGE_SQUARED_MAX = pow(30, 2);
    // cap voltage below which the indicator bar turns from green to yellow
    static constexpr uint16_t VOLTAGE_SQUARED_YELLOW = pow(20, 2);
    // cap voltage below which the indicator bar turns from yellow to orange
    static constexpr uint16_t VOLTAGE_SQUARED_ORANGE = pow(15, 2);
    // minimum displayed voltage of the supercapacitors (minimum usable voltage)
    static constexpr uint16_t VOLTAGE_SQUARED_MIN =
        pow(can::capbank::CAPACITOR_BANK_MIN_VOLTAGE, 2);

    const can::capbank::CapacitorBank *capBank;

    /**
     * Background line for the status that highlights a few states.
     */
    Tx::Graphic1Message capBankBackgroundLine;

    Tx::GraphicColor previousColor;

    /**
     * A line that shows the charge of the Capacitor Bank.
     */
    Tx::Graphic1Message capBankVoltageLevel;
    tap::arch::PeriodicMilliTimer voltageUpdateTimer;

    /**
     * A graphic that represents the current status of the Capacitor Bank.
     */
    Tx::GraphicCharacterMessage capBankTextGraphic;

    can::capbank::State previousState;
};
}  // namespace aruwsrc::control::client_display

#endif  //  CAP_BANK_INDICATOR_HPP_
