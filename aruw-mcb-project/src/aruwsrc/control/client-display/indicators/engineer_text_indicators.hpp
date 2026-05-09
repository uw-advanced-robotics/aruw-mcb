/*
 * Copyright (c) 2026-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ENGINEER_TEXT_HUD_INDICATORS_HPP_
#define ENGINEER_TEXT_HUD_INDICATORS_HPP_

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/sensors/limit_switch/limit_switch_interface.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/imu/imu_calibrate_command.hpp"
#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

namespace aruwsrc::control::client_display::indicators
{
/**
 * A list of text indicators that are displayed if a condition is met
 */

class EngineerTextIndicators : public HudIndicator, protected modm::Resumable<2>
{
public:
    /**
     * Construct a EngineerTextIndicators object.
     *
     * @param[in] drivers Global drivers instance.
     * @param[in] agitatorSubsystem Agitator used when checking if the agitator is jammed.
     * @param[in] imuCalibrateCommand IMU calibrate command used to check if the IMU is being
     * calibrated.
     * @param[in] refSerialTransmitter Transmitter to send client data
     */
    EngineerTextIndicators(
        tap::Drivers &drivers,
        const aruwsrc::control::imu::ImuCalibrateCommand &imuCalibrateCommand,
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);

    modm::ResumableResult<void> update() override final;

    void initialize() override final;
    bool checkIfSentryLow();

private:
    struct TextIndicatorData
    {
        const char *text;
        Tx::GraphicColor color;
        uint16_t x;
        uint16_t y;
        uint16_t size;
        uint16_t textWidth;
    };

    enum TextIndicators
    {
        IMU_CALIBRATING,
        NO_RECEPTACLE,
        ALIGNED_WITH_RECEPTACLE,
        NUM_TEXT_HUD_INDICATORS
    };

    static constexpr TextIndicatorData imuCalibrating =
        {"Calibrating", Tx::GraphicColor::ORANGE, 730, 840, 20, 3};
    static constexpr TextIndicatorData noReceptacle =
        {"No Receptacle", Tx::GraphicColor::ORANGE, 700, 800, 60, 10};
    static constexpr TextIndicatorData alignedWithReceptacle =
        {"Aligned with Receptacle", Tx::GraphicColor::GREEN, 700, 760, 60, 10};

    static constexpr TextIndicatorData INDICATOR_LIST[NUM_TEXT_HUD_INDICATORS] = {
        imuCalibrating,
        noReceptacle,
        alignedWithReceptacle,
    };

    Tx::GraphicCharacterMessage textHudIndicatorGraphics[NUM_TEXT_HUD_INDICATORS];

    bool states[NUM_TEXT_HUD_INDICATORS] = {false};
    bool prevStates[NUM_TEXT_HUD_INDICATORS] = {false};

    tap::Drivers &drivers;
    const aruwsrc::control::imu::ImuCalibrateCommand &imuCalibrateCommand;

    // Resumeable function thing
    int index = 0;
};

}  // namespace aruwsrc::control::client_display::indicators

#endif  // ENGINEER_TEXT_HUD_INDICATORS_HPP_
