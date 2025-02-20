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

#ifndef TEXT_HUD_INDICATORS_HPP_
#define TEXT_HUD_INDICATORS_HPP_

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/imu/imu_calibrate_command.hpp"
#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

namespace aruwsrc::control::client_display
{
/**
 * A list of text indicators that are displayed if a condition is met
 */

class TextHudIndicators : public HudIndicator, protected modm::Resumable<2>
{
public:
    /**
     * Construct a TextHudIndicators object.
     *
     * @param[in] drivers Global drivers instance.
     * @param[in] agitatorSubsystem Agitator used when checking if the agitator is jammed.
     * @param[in] imuCalibrateCommand IMU calibrate command used to check if the IMU is being
     * calibrated.
     * @param[in] validChassisCommands List of valid chassis commands.
     * @param[in] refSerialTransmitter Transmitter to send client data
     */
    TextHudIndicators(
        tap::Drivers &drivers,
        tap::control::setpoint::SetpointSubsystem &agitatorSubsystem,
        const aruwsrc::control::imu::ImuCalibrateCommand &imuCalibrateCommand,
        const std::vector<tap::control::Command *> validChassisCommands,
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);

    modm::ResumableResult<bool> sendInitialGraphics() override final;

    modm::ResumableResult<bool> update() override final;

    void initialize() override final;

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
        AGITATOR_JAMMED,
        IMU_CALIBRATING,
        NOT_SPINNING,
        NUM_TEXT_HUD_INDICATORS
    };

    static constexpr TextIndicatorData agitatorJammed =
        {"Jammed", Tx::GraphicColor::ORANGE, 1030, 840, 20, 3};
    static constexpr TextIndicatorData imuCalibrating =
        {"Calibrating", Tx::GraphicColor::ORANGE, 730, 840, 20, 3};
    static constexpr TextIndicatorData notSpinning =
        {"SPIN!", Tx::GraphicColor::PURPLISH_RED, 730, 800, 100, 10};

    static constexpr TextIndicatorData INDICATOR_LIST[NUM_TEXT_HUD_INDICATORS] = {
        agitatorJammed,
        imuCalibrating,
        notSpinning};

    Tx::GraphicCharacterMessage textHudIndicatorGraphics[NUM_TEXT_HUD_INDICATORS];

    bool states[NUM_TEXT_HUD_INDICATORS];
    bool prevStates[NUM_TEXT_HUD_INDICATORS];

    tap::Drivers &drivers;
    tap::control::setpoint::SetpointSubsystem &agitatorSubsystem;
    const aruwsrc::control::imu::ImuCalibrateCommand &imuCalibrateCommand;
    const std::vector<tap::control::Command *> validChassisCommands;

    static constexpr uint16_t JAM_TIMEOUT_MS = 1000;
    tap::arch::MilliTimeout jamTimeout;

    // Resumeable function thing
    int index = 0;
};

}  // namespace aruwsrc::control::client_display

#endif  // TEXT_HUD_INDICATORS_HPP_
