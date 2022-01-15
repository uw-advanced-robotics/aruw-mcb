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

#include <tuple>
#include <vector>

#include "tap/architecture/periodic_timer.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_ui_wrappers/boolean_drawer.hpp"
#include "tap/control/command.hpp"

#include "aruwsrc/control/agitator/agitator_subsystem.hpp"
#include "aruwsrc/control/hopper-cover/turret_mcb_hopper_cover_subsystem.hpp"
#include "aruwsrc/control/launcher/friction_wheel_subsystem.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "modm/math/geometry/polygon_2d.hpp"
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
                             modm::Resumable<5>,
                             tap::serial::RefSerialData
{
public:
    ClientDisplayCommand(
        aruwsrc::Drivers *drivers,
        ClientDisplaySubsystem *clientDisplay,
        const aruwsrc::control::TurretMCBHopperSubsystem *hopperSubsystem,
        const aruwsrc::control::launcher::FrictionWheelSubsystem *frictionWheelSubsystem,
        aruwsrc::agitator::AgitatorSubsystem *agitatorSubsystem,
        const aruwsrc::control::turret::TurretSubsystem *turretSubsystem,
        const std::vector<const tap::control::Command *> &driveCommands);

    const char *getName() const override { return "client display"; }

    void initialize() override;

    void execute() override;

    void end(bool) override {}

    bool isFinished() const override { return false; }

private:
    // general constants

    static constexpr uint16_t SCREEN_WIDTH = 1920;
    static constexpr uint16_t SCREEN_HEIGHT = 1080;
    static constexpr int32_t DELAY_PERIOD_BTWN_SENDS = 110;

    // HUD indicator related constants

    static constexpr uint16_t HUD_INDICATOR_LIST_LAYER = 0;
    static constexpr uint8_t HUD_INDICATOR_LIST_START_NAME[] = {0, 0, 0};
    static constexpr uint16_t HUD_INDICATOR_LIST_CENTER_X = 300;
    static constexpr uint16_t HUD_INDICATOR_LIST_START_Y = 600;
    static constexpr uint16_t HUD_INDICATOR_LIST_DIST_BTWN_BULLETS = 50;

    static constexpr Tx::GraphicColor HUD_INDICATOR_FILLED_COLOR = Tx::GraphicColor::GREEN;
    static constexpr uint16_t HUD_INDICATOR_WIDTH = 17;
    static constexpr uint16_t HUD_INDICATOR_RADIUS = 9;

    static constexpr uint8_t HUD_INDICATOR_STATIC_LIST_LAYER = 1;
    static constexpr Tx::GraphicColor HUD_INDICATOR_OUTLINE_COLOR = Tx::GraphicColor::BLACK;
    static constexpr uint16_t HUD_INDICATOR_OUTLINE_WIDTH = 5;
    static constexpr uint16_t HUD_INDICATOR_OUTLINE_RADIUS = 20;

    static constexpr Tx::GraphicColor HUD_INDICATOR_LABEL_COLOR = Tx::GraphicColor::YELLOW;
    static constexpr uint16_t HUD_INDICATOR_LABEL_CHAR_SIZE = 15;
    static constexpr uint16_t HUD_INDICATOR_LABEL_CHAR_LENGTH = 3;
    static constexpr uint16_t HUD_INDICATOR_LABEL_CHAR_LINE_WIDTH = 2;

    static constexpr const char *HUD_INDICATOR_LABELS[] = {
        "SYS CALIB ",
        "HOPP      ",
        "FRIC      ",
        "CV        ",
        "AGI STATUS",
#if defined(TARGET_HERO)
        "BALL RDY  ",
#endif
    };

    enum HudIndicatorIndex
    {
        SYSTEMS_CALIBRATING = 0,
        HOPPER_OPEN,
        FRICTION_WHEELS_ON,
        CV_AIM_DATA_VALID,
        AGITATOR_STATUS_HEALTHY,
#if defined(TARGET_HERO)
        BALL_READY_FOR_LAUNCHING,
#endif
        NUM_HUD_INDICATORS,
    };

    // drive command related constants

    static constexpr uint8_t DRIVE_COMMAND_NAME[] = {1, 0, 0};
    static constexpr uint16_t DRIVE_COMMAND_GRAPHIC_LAYER = 2;
    static constexpr uint16_t DRIVE_COMMAND_START_X = 100;
    static constexpr uint16_t DRIVE_COMMAND_START_Y = 850;
    static constexpr uint16_t DRIVE_COMMAND_CHAR_SIZE = 30;
    static constexpr uint16_t DRIVE_COMMAND_CHAR_LENGTH = 5;
    static constexpr uint16_t DRIVE_COMMAND_CHAR_LINE_WIDTH = 4;

    // reticle related constants

    static constexpr uint8_t RETICLE_GRAPHIC_LAYER = 3;
    static constexpr uint8_t RETICLE_LINES_START_NAME[] = {2, 0, 0};
    static constexpr uint16_t RETICLE_THICKNESS = 1;

    static constexpr int RETICLE_CENTER_X_OFFSET = 0;

    static constexpr std::tuple<float, float, Tx::GraphicColor>
        TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES[]{
            std::tuple<float, float, Tx::GraphicColor>(50, 540, Tx::GraphicColor::YELLOW),
            std::tuple<float, float, Tx::GraphicColor>(50, 530, Tx::GraphicColor::YELLOW),
            std::tuple<float, float, Tx::GraphicColor>(50, 520, Tx::GraphicColor::YELLOW),
            std::tuple<float, float, Tx::GraphicColor>(50, 510, Tx::GraphicColor::YELLOW),
            std::tuple<float, float, Tx::GraphicColor>(70, 500, Tx::GraphicColor::YELLOW),
            std::tuple<float, float, Tx::GraphicColor>(15, 440, Tx::GraphicColor::YELLOW),
            std::tuple<float, float, Tx::GraphicColor>(15, 430, Tx::GraphicColor::YELLOW),
            std::tuple<float, float, Tx::GraphicColor>(15, 420, Tx::GraphicColor::YELLOW),
            std::tuple<float, float, Tx::GraphicColor>(15, 410, Tx::GraphicColor::YELLOW),
            std::tuple<float, float, Tx::GraphicColor>(60, 400, Tx::GraphicColor::YELLOW),
            std::tuple<float, float, Tx::GraphicColor>(10, 340, Tx::GraphicColor::RED_AND_BLUE),
            std::tuple<float, float, Tx::GraphicColor>(10, 330, Tx::GraphicColor::RED_AND_BLUE),
            std::tuple<float, float, Tx::GraphicColor>(10, 320, Tx::GraphicColor::RED_AND_BLUE),
            std::tuple<float, float, Tx::GraphicColor>(10, 310, Tx::GraphicColor::RED_AND_BLUE),
            std::tuple<float, float, Tx::GraphicColor>(50, 300, Tx::GraphicColor::RED_AND_BLUE),
        };
    static constexpr size_t NUM_RETICLE_COORDINATES =
        MODM_ARRAY_SIZE(TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES);
    static constexpr Tx::GraphicColor RETICLE_HORIZONTAL_COLOR = Tx::GraphicColor::YELLOW;

    // chassis orientation constants

    static constexpr uint8_t CHASSIS_ORIENTATION_LAYER = 4;
    static constexpr uint8_t CHASSIS_ORIENTATION_START_NAME[] = {3, 0, 0};

    static constexpr uint16_t CHASSIS_CENTER_X = 1200;
    static constexpr uint16_t CHASSIS_CENTER_Y = 150;
    static constexpr uint16_t CHASSIS_HEIGHT = 140;
    static constexpr Tx::GraphicColor CHASSIS_ORIENTATION_COLOR = Tx::GraphicColor::YELLOW;
    static constexpr Tx::GraphicColor CHASSIS_BARREL_COLOR = Tx::GraphicColor::WHITE;
    static constexpr uint16_t CHASSIS_LINE_WIDTH = 100;
    static constexpr uint16_t CHASSIS_BARREL_LINE_WIDTH = 10;
    static constexpr uint16_t CHASSIS_BARREL_LENGTH = 130;

    // turret angles constants

    static constexpr uint8_t TURRET_ANGLES_LAYER = 5;
    static constexpr uint8_t TURRET_ANGLES_START_NAME[] = {4, 0, 0};

    static constexpr uint16_t TURRET_ANGLES_FONT_SIZE = 10;
    static constexpr uint16_t TURRET_ANGLES_DECIMAL_PRECISION = 1;
    static constexpr uint16_t TURRET_ANGLES_WIDTH = 2;
    static constexpr uint16_t TURRET_ANGLES_START_X = 1000;
    static constexpr uint16_t TURRET_ANGLES_START_Y = 600;
    static constexpr Tx::GraphicColor TURRET_ANGLES_COLOR = Tx::GraphicColor::ORANGE;

    // general variables

    aruwsrc::Drivers *drivers;

    /**
     * Timer used to delay between sending messages to the referee system.
     *
     * @note The maximum frequency of this timer is 10 Hz according to RM rules.
     */
    tap::arch::MilliTimeout delayTimer{DELAY_PERIOD_BTWN_SENDS};

    // HUD indicator related variables

    /**
     * Hopper subsystem that provides information about whether or not the cover is open or closed.
     */
    const aruwsrc::control::TurretMCBHopperSubsystem *hopperSubsystem;

    /**
     * Friction wheel subsystem that provides info about if they are on/off.
     */
    const aruwsrc::control::launcher::FrictionWheelSubsystem *frictionWheelSubsystem;

    /**
     * Agitator that provides info about if it is jammed or offline.
     *
     * Should be `const` but `isJammed` is not `const`.
     */
    aruwsrc::agitator::AgitatorSubsystem *agitatorSubsystem;

    /**
     * Graphic message that will represent a dot on the screen that will be present or not,
     * depending on whether or not the hopper is open or closed.
     */
    Tx::Graphic1Message hudIndicatorGraphics[NUM_HUD_INDICATORS];

    /** The object that will do the actual drawing of the hopper open indicator. */
    tap::communication::serial::ref_serial_ui_wrapeprs::BooleanDrawer
        hudIndicatorDrawers[NUM_HUD_INDICATORS];

    /** Use this index when iterating through the hudIndicatorDrawers in protothreads. */
    int hudIndicatorIndex = 0;

    /**
     * Graphics associated with the the hud indicator graphics that do not change (labels and
     * circles around the indicators).
     */
    Tx::Graphic1Message hudIndicatorStaticGraphics[NUM_HUD_INDICATORS];
    Tx::GraphicCharacterMessage hudIndicatorStaticLabelGraphics[NUM_HUD_INDICATORS];

    // drive command related variables

    /**
     * List of commands that will be checked for in the scheduler when determining which drive
     * command is being run.
     */
    std::vector<const tap::control::Command *> driveCommands;

    /** Graphic that will send the name of the current drive command. */
    Tx::GraphicCharacterMessage driveCommandMsg;

    /** Index in `driveCommands` that is currently being displayed. */
    int currDriveCommandIndex = -1;
    int prevDriveCommandIndex = -1;

    // turret reticle variables

    /**
     * Array of `Graphic5Message`s that will be used to send all of the reticle related graphics.
     * This includes all of the reticle markers from `TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES`
     * plus a verticle line to connect the reticle markers.
     */
    Tx::Graphic5Message reticleMsg[(NUM_RETICLE_COORDINATES + 1) / 5 + 1];

    /** Index used when iterating through the reticleMsg in protothreads. */
    size_t reticleIndex = 0;

    // chassis orientation variables

    const aruwsrc::control::turret::TurretSubsystem *turretSubsystem;
    modm::Vector2i chassisOrientation;
    modm::Vector2i chassisOrientationRotated;
    modm::Vector2i chassisOrientationPrev;
    Tx::Graphic2Message chassisOrientationGraphics;

    // turret pitch/yaw angles

    Tx::Graphic2Message turretAnglesGraphics;
    Tx::GraphicCharacterMessage turretAnglesLabelGraphics;
    float yaw = 0.0f;
    float pitch = 0.0f;
    float prevYaw = 0.0f;
    float prevPitch = 0.0f;

    // private functions

    bool run();

    modm::ResumableResult<bool> initializeNonblocking();

    modm::ResumableResult<bool> updateHudIndicators();
    modm::ResumableResult<bool> updateDriveCommandMsg();
    modm::ResumableResult<bool> updateChassisOrientation();
    modm::ResumableResult<bool> updateTurretAngles();

    void initializeHudIndicators();
    void initializeReticle();
    void initializeDriveCommand();
    void initializeChassisOrientation();
    void initializeTurretAngles();
};
}  // namespace aruwsrc::display

#endif  // CLIENT_DISPLAY_COMMAND_HPP_
