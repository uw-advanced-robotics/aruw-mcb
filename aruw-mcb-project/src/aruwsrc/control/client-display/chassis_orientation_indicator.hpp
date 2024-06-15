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

#ifndef CHASSIS_ORIENTATION_INDICATOR_HPP_
#define CHASSIS_ORIENTATION_INDICATOR_HPP_

#include <vector>

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

#include "aruwsrc/control/turret/robot_turret_subsystem.hpp"
#include "modm/math/geometry/vector2.hpp"
#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

namespace tap
{
class Drivers;
}

namespace aruwsrc::control::client_display
{
/**
 * Draws a little chassis graphic on the HUD that represents the actual rotation of the chassis
 * relative to the turret. A turret is drawn as a line straight up and the chassis rotates around
 * the turret.
 */
class ChassisOrientationIndicator : public HudIndicator, protected modm::Resumable<2>
{
public:
    /**
     * Construct a ClientDisplayCommand.
     *
     * @param[in] drivers Global drivers instance.
     * @param[in] refSerialTransmitter Transmitter that stores ref serial transmission state for the
     * protothread that this indicator is used in.
     * @param[in] turretSubsystem Turret used when updating chassis orientation relative to the
     * turret and to print turret angles (if turret chassis relative angles are being printed).
     */
    ChassisOrientationIndicator(
        tap::Drivers &drivers,
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
        const aruwsrc::control::turret::RobotTurretSubsystem &turretSubsystem,
        const std::vector<tap::control::Command *> avoidanceCommands);

    modm::ResumableResult<bool> sendInitialGraphics() override final;

    modm::ResumableResult<bool> update() override final;

    void initialize() override final;

private:
    /** The X location of the center of the animated chassis on the screen, in pixels. */
    static constexpr uint16_t CHASSIS_CENTER_X = 1300;
    /** The Y location of the center of the animated chassis on the screen, in pixels. */
    static constexpr uint16_t CHASSIS_CENTER_Y = 100;
    /** The length of the animated chassis, in pixels. */
    static constexpr uint16_t CHASSIS_LENGTH = 100;
    /** The width of the animated chassis, in pixels. */
    static constexpr uint16_t CHASSIS_WIDTH = 70;
    /** The color of the animated chassis when there is avoidance. */
    static constexpr Tx::GraphicColor CHASSIS_ORIENTATION_AVOIDANCE_COLOR =
        Tx::GraphicColor::YELLOW;
    /** The color of the animated chassis when there is no avoidance. */
    static constexpr Tx::GraphicColor CHASSIS_ORIENTATION_STILL_COLOR =
        Tx::GraphicColor::PURPLISH_RED;
    /** The color of the animated turret barrel in the chassis orientation graphic. */
    static constexpr Tx::GraphicColor CHASSIS_BARREL_COLOR = Tx::GraphicColor::WHITE;
    /** The width of the animated turret barrel, in pixels. */
    static constexpr uint16_t CHASSIS_BARREL_WIDTH = 10;
    /** The length of the animated turret barrel, in pixels. */
    static constexpr uint16_t CHASSIS_BARREL_LENGTH = 90;

    tap::Drivers &drivers;

    const aruwsrc::control::turret::TurretSubsystem &turretSubsystem;
    const std::vector<tap::control::Command *> avoidanceCommands;
    /**
     * Vector with origin `(0, 0)` and length CHASSIS_LENGTH / 2. The turret drawn on the screen is
     * considered to be pointing up in the y axis (of the screen). This vector can be rotated around
     * the origin by some amount to represent the rotation of the chassis orientation. A clockwise
     * rotation of the turret results in a counterclockwise rotation of the graphic (since the
     * graphic is the chassis relative to the turret).
     */
    modm::Vector2i chassisOrientation;
    /** Previous chassis orientation. Should be a local variable but cannot since it is in a
     * protothread. */
    modm::Vector2i chassisOrientationPrev;

    bool prevAvoidance;
    /**
     * Two graphics that represent chassis orientation. The first graphic is the line representing
     * the turret, and the second graphic is a thick line that represents the chassis and is rotated
     * some amount to represent chassis orientation.
     */
    Tx::Graphic2Message chassisOrientationGraphics;
};
}  // namespace aruwsrc::control::client_display

#endif  //  CHASSIS_ORIENTATION_INDICATOR_HPP_
