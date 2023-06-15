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

#include "tap/architecture/periodic_timer.hpp"
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

#include "aruwsrc/control/turret/robot_turret_subsystem.hpp"
#include "modm/math/utils/misc.hpp"

#include "hud_indicator.hpp"

namespace tap
{
class Drivers;
}

namespace aruwsrc::display::client
{
/**
 * Draws the current pitch/yaw turret angles (in degrees, in the world frame).
 */
class TurretAnglesIndicator : public HudIndicator, protected modm::Resumable<2>
{
public:
    /**
     * Construct a TurretAnglesIndicator object.
     *
     * @param[in] drivers Global drivers instance.
     * @param[in] robotTurretSubsystem Turret used when updating chassis orientation relative
     * to the turret and to print turret angles (if turret chassis relative angles are being
     * printed).
     */
    TurretAnglesIndicator(
        tap::Drivers &drivers,
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
        const aruwsrc::control::turret::RobotTurretSubsystem &robotTurretSubsystem);

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

    tap::Drivers &drivers;

    const aruwsrc::control::turret::RobotTurretSubsystem &robotTurretSubsystem;

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
    std::size_t bytesWritten = 0;
    /** Periodic timer used to regulate how often the turret angles will update. */
    tap::arch::PeriodicMilliTimer sendTurretDataTimer{TURRET_ANGLES_SEND_DATA_PERIOD};

    void updateTurretAnglesGraphicMsg();
};
}  // namespace aruwsrc::display::client

#endif  //  TURRET_ANGLES_INDICATOR_HPP_
