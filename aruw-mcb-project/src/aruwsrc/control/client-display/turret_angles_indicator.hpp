/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TURRET_ANGLES_INDICATOR_HPP_
#define TURRET_ANGLES_INDICATOR_HPP_

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "modm/math/utils/misc.hpp"

#include "hud_indicator.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::client_display
{
/**
 * Draws the current pitch/yaw turret angles (in degrees, in the world frame).
 */
class TurretAnglesIndicator : public HudIndicator
{
public:
    /**
     * Construct a TurretAnglesIndicator object.
     *
     * @param[in] drivers Global drivers instance.
     * @param[in] turretSubsystem Turret used when updating chassis orientation relative to the
     * turret and to print turret angles (if turret chassis relative angles are being printed).
     */
    TurretAnglesIndicator(
        aruwsrc::Drivers *drivers,
        const aruwsrc::control::turret::TurretSubsystem &turretSubsystem);

    modm::ResumableResult<bool> sendInitialGraphics() override final;

    modm::ResumableResult<bool> update() override final;

    void initialize() override final;

private:
    /** Minimum period between sending turret angles. */
    static constexpr uint32_t TURRET_ANGLES_SEND_DATA_PERIOD = 250;
    /** Font size of the turret angles. */
    static constexpr uint16_t TURRET_ANGLES_CHAR_SIZE = 10;
    /** Number of decimal points that the displayed turret angle data will have.. */
    static constexpr uint16_t TURRET_ANGLES_DECIMAL_POINTS = 2;
    static constexpr int TURRET_ANGLES_DECIMAL_PRECISION =
        modm::pow(10, TURRET_ANGLES_DECIMAL_POINTS);
    /** The character line width of the turret angles text. */
    static constexpr uint16_t TURRET_ANGLES_CHAR_WIDTH = 2;
    /** Starting X coordinate of the turret angles message, in pixels. Location where turret
     * floating point angles will be displayed at. The labels associated with the numbers will be to
     * the left of this. */
    static constexpr uint16_t TURRET_ANGLES_START_X = 1430;
    /** Starting Y coordinate of the turret angles message, in pixels. */
    static constexpr uint16_t TURRET_ANGLES_START_Y = 460;
    /** Color of the turret angles message. */
    static constexpr Tx::GraphicColor TURRET_ANGLES_COLOR = Tx::GraphicColor::ORANGE;

    aruwsrc::Drivers *drivers;

    const aruwsrc::control::turret::TurretSubsystem &turretSubsystem;

    /** Character graphic containing turret pitch/yaw angles. */
    Tx::GraphicCharacterMessage turretAnglesGraphic;
    /** Character graphic containing labeles "pitch" and "yaw", situated next to
     * turretAnglesGraphic. */
    Tx::GraphicCharacterMessage turretAnglesLabelGraphics;
    /** Current turret yaw value, in degrees. Should be a local variable, but since it's in a
     * protothread it can't be local. */
    float yaw = 0.0f;
    /** Current turret pitch angle value, in degrees. Should be a local variable, but since it's in
     * a protothread it can't be local. */
    float pitch = 0.0f;
    /** Previous turret yaw value used to determine if yaw turret angle has changed. */
    float prevYaw = 0.0f;
    /** Previous turret pitch value used to determine if pitch turret angle has changed. */
    float prevPitch = 0.0f;
    /** Mumber of bytes written when writing the angle data graphic. Should be a local variable, but
     * since it's in a protothread it can't be local. */
    int bytesWritten = 0;
    /** Periodic timer used to regulate how often the turret angles will update. */
    tap::arch::PeriodicMilliTimer sendTurretDataTimer{TURRET_ANGLES_SEND_DATA_PERIOD};
};
}  // namespace aruwsrc::control::client_display

#endif  //  TURRET_ANGLES_INDICATOR_HPP_
