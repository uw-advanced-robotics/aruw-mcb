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

#ifndef MATRIX_HUD_INDICATORS_HPP_
#define MATRIX_HUD_INDICATORS_HPP_

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

#include "aruwsrc/control/agitator/multi_shot_cv_command_mapping.hpp"
#include "aruwsrc/control/chassis/beyblade_command.hpp"
#include "aruwsrc/control/chassis/chassis_autorotate_command.hpp"
#include "aruwsrc/control/chassis/chassis_drive_command.hpp"
#include "aruwsrc/control/chassis/chassis_imu_drive_command.hpp"
#include "aruwsrc/control/governor/cv_on_target_governor.hpp"
#include "aruwsrc/control/hopper-cover/turret_mcb_hopper_cover_subsystem.hpp"
#include "aruwsrc/control/launcher/friction_wheel_subsystem.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"

#include "hud_indicator.hpp"

namespace tap
{
class Drivers;
}

namespace aruwsrc::control::client_display
{
/**
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
class MatrixHudIndicators : public HudIndicator, protected modm::Resumable<2>
{
public:
    /**
     * Construct a MatrixHudIndicators object.
     *
     * @param[in] drivers Global drivers instance.
     * @param[in] hopperSubsystem Hopper used when checking if the hopper is open/closed. A pointer
     * that may be nullptr if no hopper exists.
     * @param[in] frictionWheelSubsystem Friction wheels used when checking if the friction wheels
     * are on or off.
     * @param[in] multiShotHandler Shot handler, used to determine which shooting mode the agitator
     * is in. May be nullptr, if so multi shot mode defaults to single shot (as displayed on the
     * HUD).
     * @param[in] cvOnTargetGovernor This governor is checked to see whether or not projectile
     * launching is being gated by CV. May be nullptr if no governor exists in the system (in which
     * case it is assumed that no CV launch limiting is being performed).
     * @param[in] chassisBeybladeCmd May be nullptr. If nullptr the chassis beyblade command will
     * never be selected as the current chassis command in the HUD.
     * @param[in] chassisAutorotateCmd May be nullptr. If nullptr the chassis autorotate command
     * will never be selected as the current chassis command in the HUD.
     * @param[in] chassisImuDriveCommand May be nullptr. If nullptr the chassis IMU drive command
     * will never be selected as the current chassis command in the HUD.
     */
    MatrixHudIndicators(
        tap::Drivers &drivers,
        aruwsrc::serial::VisionCoprocessor &visionCoprocessor,
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
        const aruwsrc::control::TurretMCBHopperSubsystem *hopperSubsystem,
        const aruwsrc::control::launcher::FrictionWheelSubsystem &frictionWheelSubsystem,
        const aruwsrc::control::turret::TurretSubsystem &turretSubsystem,
        const aruwsrc::control::agitator::MultiShotCvCommandMapping *multiShotHandler,
        const aruwsrc::control::governor::CvOnTargetGovernor *cvOnTargetGovernor,
        const aruwsrc::chassis::BeybladeCommand *chassisBeybladeCmd,
        const aruwsrc::chassis::ChassisAutorotateCommand *chassisAutorotateCmd,
        const aruwsrc::chassis::ChassisImuDriveCommand *chassisImuDriveCommand);

    modm::ResumableResult<bool> sendInitialGraphics() override final;

    modm::ResumableResult<bool> update() override final;

    void initialize() override final;

private:
#if defined(ALL_STANDARDS)
#define DISPLAY_FIRING_MODE
#endif

    /** The color of the title row of the matrix HUD indicator. */
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
        MATRIX_HUD_INDICATOR_CHAR_SIZE;

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
        SHOOTER_STATE,
#if defined(DISPLAY_FIRING_MODE)
        /** The current projectile launching state (single, burst, full auto). TODO */
        FIRING_MODE,
#endif
        /** The current state of CV. */
        CV_STATUS,
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
            {"CHAS", "BEYB\nFLLW\nMIMU"},
            {"SHOT", "REDY\nLOAD\nFOFF"},
#if defined(DISPLAY_FIRING_MODE)
            {"FIRE", "SNGL\nCONST\n10Hz\n20Hz\nMAX\n"},
#endif
            {"CV  ", "GATE\nNOGT\nOFFL"}
        };

    /** Number of possible chassis states associated with MatrixHUDIndicatorIndex::CHASSIS_STATE. */
    static constexpr int NUM_CHASSIS_STATES = 4;

    /** Enum representing different states that the shooting mechanism can be in. Corresponds to
     * MATRIX_HUD_INDICATOR_TITLES_AND_LABELS[FIRING_MODE]. */
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

    /** Enum representing different states that the CV can be in. Corresponds to
     * MATRIX_HUD_INDICATOR_TITLES_AND_LABELS[CV_STATUS]. */
    enum class CVStatus
    {
        /** The vision coprocessor is connected and is in the mode where vision gates projectiles
           from launching. */
        VISION_COPROCESSOR_GATED_PROJECTILE_LAUNCH = 0,
        /** The vision coprocessor is connected and is freely allowing the user to fire
         * projectiles. */
        VISION_COPROCESSOR_NO_PROJECTILE_GATING,
        /** The vision coprocessor is offline. */
        VISION_COPROCESSOR_OFFLINE,
    };

    tap::Drivers &drivers;

    aruwsrc::serial::VisionCoprocessor &visionCoprocessor;

    const aruwsrc::control::TurretMCBHopperSubsystem *hopperSubsystem;

    const aruwsrc::control::launcher::FrictionWheelSubsystem &frictionWheelSubsystem;

    const aruwsrc::control::turret::TurretSubsystem &turretSubsystem;

    const aruwsrc::control::agitator::MultiShotCvCommandMapping *multiShotHandler;

    const aruwsrc::control::governor::CvOnTargetGovernor *cvOnTargetGovernor;

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

    void updateIndicatorState();

    /**
     * Converts a 0-based index to a physical y-coordinate on the screen that represents the
     * location of the indicator on the screen.
     */
    static inline uint16_t getIndicatorYCoordinate(int index)
    {
        return MATRIX_HUD_INDICATOR_LABELS_START_Y -
               CHARACTER_LINE_SPACING * index * MATRIX_HUD_INDICATOR_CHAR_SIZE -
               MATRIX_HUD_INDICATOR_CHAR_SIZE - MATRIX_HUD_INDICATOR_SELECTOR_BOX_WIDTH - 1;
    }
};
}  // namespace aruwsrc::control::client_display

#endif  //  MATRIX_HUD_INDICATORS_HPP_
