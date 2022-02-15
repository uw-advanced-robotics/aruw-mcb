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

#include <array>
#include <tuple>

#include "tap/architecture/periodic_timer.hpp"
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/control/command.hpp"

#include "aruwsrc/control/agitator/agitator_subsystem.hpp"
#include "aruwsrc/control/chassis/beyblade_command.hpp"
#include "aruwsrc/control/chassis/chassis_autorotate_command.hpp"
#include "aruwsrc/control/chassis/chassis_drive_command.hpp"
#include "aruwsrc/control/chassis/chassis_imu_drive_command.hpp"
#include "aruwsrc/control/hopper-cover/turret_mcb_hopper_cover_subsystem.hpp"
#include "aruwsrc/control/imu/imu_calibrate_command.hpp"
#include "aruwsrc/control/launcher/friction_wheel_subsystem.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "modm/math/geometry/polygon_2d.hpp"
#include "modm/math/utils/misc.hpp"
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

/**
 * A command that controls what is displayed on the RoboMaster client's interactive HUD.
 *
 * The following graphic features are supported:
 * - Static reticle.
 * - Simulated chassis to represent chassis location.
 * - A list of boolean indicators.
 * - A table of selection indicators (where the graphic can be in one of multiple states).
 * - Turret pitch/yaw angles.
 *
 * @note Only a single ClientDisplayCommand should be instantiated. If more than 1 is instantiated,
 * this will lead to undefined behavior.
 */
class ClientDisplayCommand : public tap::control::Command,
                             ::modm::pt::Protothread,
                             modm::Resumable<5>,
                             tap::communication::serial::RefSerialData
{
public:
    /**
     * Construct a ClientDisplayCommand.
     *
     * @param[in] drivers Global drivers instance.
     * @param[in] clientDisplay The client display subsystem associated with the command.
     * @param[in] hopperSubsystem Hopper used when checking if the hopper is open/closed. A pointer
     * that may be nullptr if no hopper exists.
     * @param[in] frictionWheelSubsystem Friction wheels used when checking if the friction wheels
     * are on or off.
     * @param[in] agitatorSubsystem Agitator used when checking if the agitator is jammed.
     * @param[in] turretSubsystem Turret used when updating chassis orientation relative to the
     * turret and to print turret angles (if turret chassis relative angles are being printed).
     * @param[in] chassisBeybladeCmd May be nullptr. If nullptr the chassis beyblade command will
     * never be selected as the current chassis command.
     * @param[in] chassisAutorotateCmd May be nullptr. If nullptr the chassis autorotate command
     * will never be selected as the current chassis command.
     * @param[in] chassisImuDriveCommand May be nullptr. If nullptr the chassis IMU drive command
     * will never be selected as the current chassis command.
     * @param[in] chassisDriveCmd May be nullptr. If nullptr the chassis drive command will never be
     * selected as the current chassis command.
     */
    ClientDisplayCommand(
        aruwsrc::Drivers *drivers,
        ClientDisplaySubsystem *clientDisplay,
        const aruwsrc::control::TurretMCBHopperSubsystem *hopperSubsystem,
        const aruwsrc::control::launcher::FrictionWheelSubsystem &frictionWheelSubsystem,
        aruwsrc::agitator::AgitatorSubsystem &agitatorSubsystem,
        const aruwsrc::control::turret::TurretSubsystem &turretSubsystem,
        const aruwsrc::control::imu::ImuCalibrateCommand &imuCalibrateCommand,
        const aruwsrc::chassis::BeybladeCommand *chassisBeybladeCmd,
        const aruwsrc::chassis::ChassisAutorotateCommand *chassisAutorotateCmd,
        const aruwsrc::chassis::ChassisImuDriveCommand *chassisImuDriveCommand,
        const aruwsrc::chassis::ChassisDriveCommand *chassisDriveCmd);

    const char *getName() const override { return "client display"; }

    void initialize() override;

    void execute() override;

    void end(bool) override {}

    bool isFinished() const override { return false; }

private:
    // general constants

    /*
     * Note that absolute X/Y pixel coordinates are measured from the bottom left side of the
     * screen, X is increasing from left to right and Y is increasing from bottom to top.
     */

    /** Width of the screen, in pixels. */
    static constexpr uint16_t SCREEN_WIDTH = 1920;
    /** Height of the screen, in pixels. */
    static constexpr uint16_t SCREEN_HEIGHT = 1080;
    /** Unless you have a particular reason to do otherwise, place all graphics in this layer. */
    static constexpr uint8_t DEFAULT_GRAPHIC_LAYER = 0;
    /** The line spacing of the characters in a characterGraphicMessage if a `\n` character is
     * inserted. */
    static constexpr float CHARACTER_LINE_SPACING = 1.5f;

    // Boolean HUD indicator related constants

    /** X pixel coordinate that the boolean hud indicator circles will be centered around. */
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_LIST_CENTER_X = 280;
    /** Starting y value where boolean hud indicator circles will start. The top most circle in the
     * list will be centered around this point. Subsequent circles will be below this y pixel value.
     */
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_LIST_START_Y = 775;
    /** Distance between the center of the the boolean indicators, in the y direction. */
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_LIST_DIST_BTWN_BULLETS = 50;
    /** The line width of the indicator circles. Should be approximately twice
     * BOOLEAN_HUD_INDICATOR_RADIUS if you want the circle to be filled completely. There is no way
     * to fill in circle grpahics, so the next best thing is to make the line width very large. */
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_WIDTH = 17;
    /** The radius of the boolean indicator circles. */
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_RADIUS = 9;
    /** The color that the HUD indicator will be outlined in. */
    static constexpr Tx::GraphicColor BOOLEAN_HUD_INDICATOR_OUTLINE_COLOR = Tx::GraphicColor::BLACK;
    /** The width of the boolean HUD indicator's outline circle. */
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_OUTLINE_WIDTH = 5;
    /** The radius of the boolean HUD indicator outline circle. */
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_OUTLINE_RADIUS = 20;
    /** The color of the textual label associated with the boolean HUD indicators. */
    static constexpr Tx::GraphicColor BOOLEAN_HUD_INDICATOR_LABEL_COLOR = Tx::GraphicColor::ORANGE;
    /** The character size of the textual label associated with the boolean HUD indicators */
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_LABEL_CHAR_SIZE = 15;
    /** The character line width of the textual label associated with the boolean HUD indicators */
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_LABEL_CHAR_LINE_WIDTH = 3;

    /** Tuple that contains the name of the boolean indicator, the color when the indicator is
     * `true` or on, and the color when the indicator is `false` or off. */
    using BooleanHUDIndicatorTuple = std::tuple<const char *, Tx::GraphicColor, Tx::GraphicColor>;

    /**
     * Enum containing all boolean HUD indicators.
     */
    enum BooleanHUDIndicatorIndex
    {
        /** Indicates systems (such as the IMU) are calibrating. */
        SYSTEMS_CALIBRATING = 0,
        /** Indicates the agitator is online and not jammed. */
        AGITATOR_STATUS_HEALTHY,
        /** Indicates that the vision system is online and a target has been acquired. */
        CV_AIM_DATA_VALID,
        /** Should always be the last value, the number of enum values listed in this enum (as such,
           the first element in this enum should be 0 and subsequent ones should increment by 1
           each). */
        NUM_BOOLEAN_HUD_INDICATORS,
    };

    /**
     * List of `NUM_BOOLEAN_HUD_INDICATORS` `BooleanHUDIndicatorTuple`s. Each item associated with
     * an enum value above.
     */
    static constexpr BooleanHUDIndicatorTuple
        BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[NUM_BOOLEAN_HUD_INDICATORS]{
            BooleanHUDIndicatorTuple(
                "SYS CALIB ",
                Tx::GraphicColor::PURPLISH_RED,
                Tx::GraphicColor::GREEN),
            BooleanHUDIndicatorTuple(
                "AGI ",
                Tx::GraphicColor::GREEN,
                Tx::GraphicColor::PURPLISH_RED),
            BooleanHUDIndicatorTuple(
                "CV ",
                Tx::GraphicColor::GREEN,
                Tx::GraphicColor::PURPLISH_RED),
        };

    // matrix HUD indicator related constants

    /*
     * The matrix HUD will display a matrix of possible robot states. Each column is a different
     * robot feature (for example chassis, turret, firing system, etc.). A box circling a particular
     * robot state indicates the robot is in that particular state.
     *
     * The matrix indicator looks something like this:
     *
     * ```
     *   CHAS   SHOT
     *  +----+
     *  |BEYB|  REDY
     *  +----+ +----+
     *   FLLW  |LOAD|
     *         +----+
     * ```
     *
     * In the example above, the chassis is beyblading and the launcher is loading.
     */

    /** The color of the title row of the matrix HUD indicator */
    static constexpr Tx::GraphicColor MATRIX_HUD_INDICATOR_TITLE_COLOR = Tx::GraphicColor::GREEN;
    /** The color of the labels in the HUD matrix. */
    static constexpr Tx::GraphicColor MATRIX_HUD_INDICATOR_LABELS_COLOR = Tx::GraphicColor::ORANGE;
    /** The color of the selector box in the HUD matrix. */
    static constexpr Tx::GraphicColor MATRIX_HUD_INDICATOR_SELECTOR_BOX_COLOR =
        Tx::GraphicColor::PINK;
    /** The line width of the selector box. */
    static constexpr uint16_t MATRIX_HUD_INDICATOR_SELECTOR_BOX_WIDTH = 2;
    /** The character size of the matrix HUD indicator labels and title. */
    static constexpr uint16_t MATRIX_HUD_INDICATOR_CHAR_SIZE = 14;
    /** The character line width of the matrix HUD indicator labels and title. */
    static constexpr uint16_t MATRIX_HUD_INDICATOR_CHAR_LINE_WIDTH = 3;
    /** The horizontal distance between columns in the HUD matrix, the distance between the
     * rightmost character in one column and the leftmost character in the column next to it. */
    static constexpr uint8_t MATRIX_HUD_INDICATOR_DIST_BTWN_INDICATOR_COLS =
        2 * MATRIX_HUD_INDICATOR_CHAR_SIZE;

    /** The starting X point where the matrix HUD indicator will be situated, in pixels. */
    static constexpr uint16_t MATRIX_HUD_INDICATOR_START_X = 350;
    /** The starting Y point for the title row of the matrix indicator, in pixels. */
    static constexpr uint16_t MATRIX_HUD_INDICATOR_TITLE_START_Y = 775;
    /** The starting Y point for the labels situated below the row of titles, in pixels. */
    static constexpr uint16_t MATRIX_HUD_INDICATOR_LABELS_START_Y =
        MATRIX_HUD_INDICATOR_TITLE_START_Y - 2 * MATRIX_HUD_INDICATOR_CHAR_SIZE;
    /** Enum values representing various columns in the matrix indicator. */
    enum MatrixHUDIndicatorIndex
    {
        /** The current command controlling the chassis. */
        CHASSIS_STATE = 0,
        /** The current reloading and flywheel state of the firing system. */
        FLYWHEEL_AND_HOPPER_STATE,
        /** The current projectile launching state (single, burst, full auto). TODO */
        SHOOTER_STATE,
        /** Should always be the last value, the number of enum values listed in this enum (as such,
           the first element in this enum should be 0 and subsequent ones should increment by 1
           each). */
        NUM_MATRIX_HUD_INDICATORS,
    };
    /** The number of characters in a matrix indicator title (or label). */
    static constexpr uint8_t MATRIX_HUD_INDICATOR_TITLE_WIDTH = 4;
    /** List of titles and their associated labels. Labels should contain a newline characters
     * between each other. All titles and labels should be MATRIX_HUD_INDICATOR_TITLE_WIDTH
     * characters long, otherwise the graphics won't align properly when these strings are drawn.
     * None of the below strings should be > 30 characters, otherwise they will be cut off when
     * drawing. */
    static constexpr const char
        *MATRIX_HUD_INDICATOR_TITLES_AND_LABELS[NUM_MATRIX_HUD_INDICATORS][2] = {
            {"CHAS", "BEYB\nFLLW\nMIMU\nMNOR"},
            {"SHOT", "REDY\nLOAD\nFOFF"},
            {"FIRE", "SNGL\nBRST\nFULL"},
        };

    /** Number of possible chassis states associated with MatrixHUDIndicatorIndex::CHASSIS_STATE. */
    static constexpr int NUM_CHASSIS_STATES = 4;

    /** Enum representing different states that the shooting mechanism can be in. Corresponds to
     * MATRIX_HUD_INDICATOR_TITLES_AND_LABELS[SHOOTER_STATE]. */
    enum class ShooterState
    {
        /** The launcher is ready to fire. Any hoppers are sealed, the flywheels are on, and if
         * there is some sensor indicating a ball is ready to be launched the sensor indicates the
         * ball is ready. */
        READY_TO_FIRE = 0,
        /** The launcher is not necessarily ready to fire. This could be because the hopper is open
           or there there are no balls ready to be launched. */
        LOADING,
        /** The flywheels are off, indicating the shooter is not ready to fire. */
        FLYWHEELS_OFF,
    };

    // reticle related constants

    /** Line thickness of the reticle, in pixels. */
    static constexpr uint16_t RETICLE_THICKNESS = 1;
    /** Number of pixels to offset the reticle from the horizontal center of the screen. */
    static constexpr int RETICLE_CENTER_X_OFFSET = -5;

    /** Tuple representing a possible horizontal reticle line. The first element is the pixel width
     * of the line, second is Y location of the line (in pixels), third is the color of the reticle
     * line. */
    using ReticleTuple = std::tuple<int16_t, int16_t, Tx::GraphicColor>;

    static constexpr ReticleTuple TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES[]{
        ReticleTuple(50, 435, Tx::GraphicColor::YELLOW),  // 1 m
        ReticleTuple(50, 430, Tx::GraphicColor::YELLOW),
        ReticleTuple(70, 425, Tx::GraphicColor::YELLOW),
        ReticleTuple(50, 420, Tx::GraphicColor::YELLOW),
        ReticleTuple(50, 415, Tx::GraphicColor::YELLOW),
        ReticleTuple(40, 410, Tx::GraphicColor::ORANGE),  // 3 m
        ReticleTuple(40, 405, Tx::GraphicColor::ORANGE),
        ReticleTuple(60, 400, Tx::GraphicColor::ORANGE),
        ReticleTuple(40, 395, Tx::GraphicColor::ORANGE),
        ReticleTuple(40, 390, Tx::GraphicColor::ORANGE),
        ReticleTuple(10, 370, Tx::GraphicColor::YELLOW),  // 5 m
        ReticleTuple(10, 365, Tx::GraphicColor::YELLOW),
        ReticleTuple(30, 360, Tx::GraphicColor::YELLOW),
        ReticleTuple(10, 355, Tx::GraphicColor::YELLOW),
        ReticleTuple(10, 350, Tx::GraphicColor::YELLOW),
    };
    /** Size of TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES (so its easier to understand when used
     * in context). */
    static constexpr size_t NUM_RETICLE_COORDINATES =
        MODM_ARRAY_SIZE(TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES);
    /** The color of the verticle line that connects the horizontal reticle lines. */
    static constexpr Tx::GraphicColor RETICLE_VERTICAL_COLOR = Tx::GraphicColor::YELLOW;

    // chassis orientation constants

    /** The X location of the center of the animated chassis on the screen, in pixels. */
    static constexpr uint16_t CHASSIS_CENTER_X = 1300;
    /** The Y location of the center of the animated chassis on the screen, in pixels. */
    static constexpr uint16_t CHASSIS_CENTER_Y = 100;
    /** The length of the animated chassis, in pixels. */
    static constexpr uint16_t CHASSIS_LENGTH = 100;
    /** The width of the animated chassis, in pixels. */
    static constexpr uint16_t CHASSIS_WIDTH = 70;
    /** The color of the animated chassis. */
    static constexpr Tx::GraphicColor CHASSIS_ORIENTATION_COLOR = Tx::GraphicColor::YELLOW;
    /** The color of the animated turret barrel in the chassis orientation graphic. */
    static constexpr Tx::GraphicColor CHASSIS_BARREL_COLOR = Tx::GraphicColor::WHITE;
    /** The width of the animated turret barrel, in pixels. */
    static constexpr uint16_t CHASSIS_BARREL_WIDTH = 10;
    /** The length of the animated turret barrel, in pixels. */
    static constexpr uint16_t CHASSIS_BARREL_LENGTH = 90;

    // turret angles constants

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
    /** Starting X coordinate of the turret angles mssage, in pixels. Location where turret floating
     * point angles will be displayed at. The labels associated with the numbers will be to the left
     * of this. */
    static constexpr uint16_t TURRET_ANGLES_START_X = 1430;
    /** Starting Y coordinate of the turret angles message, in pixels. */
    static constexpr uint16_t TURRET_ANGLES_START_Y = 460;
    /** Color of the turret angles message. */
    static constexpr Tx::GraphicColor TURRET_ANGLES_COLOR = Tx::GraphicColor::ORANGE;

    // general variables

    aruwsrc::Drivers *drivers;

    /**
     * Timer used to delay between sending messages to the referee system.
     *
     * @note The maximum frequency of this timer is 10 Hz according to RM rules.
     */
    tap::arch::MilliTimeout delayTimer;

    /**
     * Used to determine the current name of the graphic, queried by `getUnusedListName`. Max value
     * is 2^24-1.
     */
    uint32_t currListName = 0;

    // Boolean HUD indicator related variables

    /**
     * Hopper subsystem that provides information about whether or not the cover is open or closed.
     */
    const aruwsrc::control::TurretMCBHopperSubsystem *hopperSubsystem;

    /**
     * Friction wheel subsystem that provides info about if they are on/off.
     */
    const aruwsrc::control::launcher::FrictionWheelSubsystem &frictionWheelSubsystem;

    /**
     * Agitator that provides info about if it is jammed or offline.
     *
     * Should be `const` but `isJammed` is not `const`.
     */
    aruwsrc::agitator::AgitatorSubsystem &agitatorSubsystem;

    /**
     * ImuCalbirateCommand that provides information about if the IMUs are being calibrated.
     */
    const aruwsrc::control::imu::ImuCalibrateCommand &imuCalibrateCommand;

    /**
     * Graphic message that will represent a dot on the screen that will be present or not,
     * depending on whether or not the hopper is open or closed.
     */
    Tx::Graphic1Message booleanHudIndicatorGraphics[NUM_BOOLEAN_HUD_INDICATORS];

    /** The objects that will do the actual drawing of the hopper open indicator. */
    tap::communication::referee::BooleanHUDIndicator
        booleanHudIndicatorDrawers[NUM_BOOLEAN_HUD_INDICATORS];

    /** Use this index when iterating through the  booleanHudIndicatorDrawers in protothreads. */
    int booleanHudIndicatorIndex = 0;

    /**
     * Graphics associated with the the hud indicator graphics that do not change (labels and
     * circles around the indicators).
     */
    Tx::Graphic1Message booleanHudIndicatorStaticGraphics[NUM_BOOLEAN_HUD_INDICATORS];
    Tx::GraphicCharacterMessage booleanHudIndicatorStaticLabelGraphics[NUM_BOOLEAN_HUD_INDICATORS];

    // position selection HUD indicator related variables

    /**
     * List of commands that will be checked for in the scheduler when determining which drive
     * command is being run.
     */
    std::array<const tap::control::Command *, NUM_CHASSIS_STATES> driveCommands;

    /** Index in `driveCommands` that is currently being displayed. Start at -1, indicating no drive
     * command has been selected and displayed. */
    int currDriveCommandIndex = -1;

    /** Array of graphic messages to be used by the `matrixHudIndicatorDrawers`. */
    Tx::Graphic1Message matrixHudIndicatorGraphics[NUM_MATRIX_HUD_INDICATORS];

    /**
     * Array of character graphic messages. Indices `[0, NUM_MATRIX_HUD_INDICATORS)` contain
     * individual label columns not including the title (i.e.
     * `MATRIX_HUD_INDICATOR_TITLES_AND_LABELS[:][1]`). index `NUM_MATRIX_HUD_INDICATORS` contains
     * the titles. The titles are separate character message from the labels because their color is
     * different.
     */
    Tx::GraphicCharacterMessage matrixHudLabelAndTitleGraphics[NUM_MATRIX_HUD_INDICATORS + 1];

    /** Array of `StateHUDIndicator`s used to update the position of the box that will be circling
     * one of the labels in `MATRIX_HUD_INDICATOR_TITLES_AND_LABELS`. */
    tap::communication::referee::StateHUDIndicator<uint16_t>
        matrixHudIndicatorDrawers[NUM_MATRIX_HUD_INDICATORS];

    /** Index used when iterating through `matrixHudIndicatorDrawers` in a protothread. */
    int matrixHudIndicatorIndex = 0;

    // turret reticle variables

    /**
     * Array of `Graphic5Message`s that will be used to send all of the reticle related graphics.
     * This includes all of the reticle markers from `TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES`
     * plus a verticle line to connect the reticle markers.
     *
     * This is an array of `Graphic5Message`s. There are NUM_RETICLE_COORDINATES + 1 reticle
     * graphics that must be fit into N Graphic5Messages. Each Graphic5Message can hold 5 reticle
     * graphics. Thus there are (NUM_RETICLE_COORDINATES + 1)/5 Graphic5Message structs rounded up
     * to the nearest whole number required to fit the reticle graphics.
     */
    Tx::Graphic5Message reticleMsg[tap::algorithms::ceil((NUM_RETICLE_COORDINATES + 1) / 5)];

    /** Index used when iterating through the reticleMsg in protothreads. */
    size_t reticleIndex = 0;

    // chassis orientation variables

    const aruwsrc::control::turret::TurretSubsystem &turretSubsystem;
    /**
     * Vector with origin `(0, 0)`. The turret drawn on the screen is considered to be pointing up
     * in the y axis (of the screen). This vector can be rotated around the origin by some amount to
     * represent the rotation of the chassis orientation. A clockwise rotation of the turret results
     * in a clockwise rotation of the graphic (the graphic is the chassis relative to the turret).
     */
    modm::Vector2i chassisOrientation;
    /** Previous chassis orientation. Should be a local variable but cannot since it is in a
     * protothread. */
    modm::Vector2i chassisOrientationPrev;
    /**
     * 2 graphics that represent chassis orientation. First graphic is the line representing the
     * turret and the second graphic is a thick line that represents the chassis and is rotated some
     * amount to represent chassis orientation.
     */
    Tx::Graphic2Message chassisOrientationGraphics;

    // turret pitch/yaw angles

    /** Character graphic containing turret pitch/yaw angles. */
    Tx::GraphicCharacterMessage turretAnglesGraphic;
    /** Character graphic containing labeles "pitch" and "yaw", situated next to
     * turretAnglesGraphic. */
    Tx::GraphicCharacterMessage turretAnglesLabelGraphics;
    /** Local variable but is in a protothread, so must be not local, current turret yaw value, in
     * degrees. */
    float yaw = 0.0f;
    /** Local variable but is in a protothread, so must be not local, current turret pitch angle
     * value, in degrees. */
    float pitch = 0.0f;
    /** Previous turret yaw value used to determine if yaw turret angle has changed. */
    float prevYaw = 0.0f;
    /** Previous turret pitch value used to determine if pitch turret angle has changed. */
    float prevPitch = 0.0f;
    /** Local variable but is in a protothread, so must be not local...number of bytes written when
     * writing the angle data graphic. */
    int bytesWritten = 0;
    /** Periodic timer used to regulate how often the turret angles will update. */
    tap::arch::PeriodicMilliTimer sendTurretDataTimer{TURRET_ANGLES_SEND_DATA_PERIOD};

    // private functions

    bool run();

    /**
     * Sends all graphics for the first time. Then, next time a graphic needs to be changed you only
     * have to modify it, which results in less latency on the client UI side.
     */
    modm::ResumableResult<bool> sendInitialGraphics();
    modm::ResumableResult<bool> updateBooleanHudIndicators();
    modm::ResumableResult<bool> updatePositionSelectorHudIndicators();
    modm::ResumableResult<bool> updateChassisOrientation();
    modm::ResumableResult<bool> updateTurretAngles();

    void initializeBooleanHudIndicators();
    void initializePositionHudIndicators();
    void initializeReticle();
    void initializeChassisOrientation();
    void initializeTurretAngles();

    void updatePositionSelectionHudIndicatorState();

    /**
     * Resets the list name generator so the next time it is queried via `getUnusedListName`, the
     * function returns {0, 0, 0}.
     */
    void resetListNameGenerator();

    /**
     * Graphics must have unique 3 byte names. Utility function for getting a list name that is
     * currently unused. Use this function exclusively to avoid graphic name clashes.
     *
     * If no list names are available (all are in use), won't set the listName and will raise an
     * error.
     *
     * @param[out] listName Array to put an unused list name in.
     */
    void getUnusedListName(uint8_t listName[3]);
};
}  // namespace aruwsrc::display

#endif  // CLIENT_DISPLAY_COMMAND_HPP_
