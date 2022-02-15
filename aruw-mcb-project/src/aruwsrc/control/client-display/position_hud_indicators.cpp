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

#include "position_hud_indicators.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "aruwsrc/drivers.hpp"

using namespace tap::communication::referee;
using namespace tap::communication::serial;
using namespace tap::algorithms;

namespace aruwsrc::control::client_display
{
/**
 * Updates the `graphic`'s `startY` to the specified `location`. Updates `endY` such that the
 * distance between `endY` and `startY` are maintained. Thus, a graphic's position in the Y
 * direction may be translated up or down using this function
 *
 * @note By convention expected that `endY > startY`. If this is not true, this function does
 * nothing.
 */
static inline void updateGraphicYLocation(
    uint16_t location,
    RefSerialData::Tx::Graphic1Message *graphic)
{
    if (graphic->graphicData.endY < graphic->graphicData.startY)
    {
        return;
    }

    uint16_t startYDiff = graphic->graphicData.endY - graphic->graphicData.startY;
    graphic->graphicData.startY = location;
    graphic->graphicData.endY = location + startYDiff;
}

PositionHudIndicators::PositionHudIndicators(
    aruwsrc::Drivers *drivers,
    const aruwsrc::control::launcher::FrictionWheelSubsystem &frictionWheelSubsystem,
    const aruwsrc::chassis::BeybladeCommand *chassisBeybladeCmd,
    const aruwsrc::chassis::ChassisAutorotateCommand *chassisAutorotateCmd,
    const aruwsrc::chassis::ChassisImuDriveCommand *chassisImuDriveCommand,
    const aruwsrc::chassis::ChassisDriveCommand *chassisDriveCmd)
    : drivers(drivers),
      frictionWheelSubsystem(frictionWheelSubsystem),
      driveCommands{
          chassisBeybladeCmd,
          chassisAutorotateCmd,
          chassisImuDriveCommand,
          chassisDriveCmd,
      },
      matrixHudIndicatorDrawers{
          StateHUDIndicator<uint16_t>(
              drivers,
              &matrixHudIndicatorGraphics[CHASSIS_STATE],
              updateGraphicYLocation,
              false),
          StateHUDIndicator<uint16_t>(
              drivers,
              &matrixHudIndicatorGraphics[FLYWHEEL_AND_HOPPER_STATE],
              updateGraphicYLocation,
              false),
          StateHUDIndicator<uint16_t>(
              drivers,
              &matrixHudIndicatorGraphics[SHOOTER_STATE],
              updateGraphicYLocation,
              false),
      }
{
}

modm::ResumableResult<bool> PositionHudIndicators::sendInitialGraphics()
{
    RF_BEGIN(0);

    // send all matrix HUD indicator-related graphics (the labels, title, and boxes)
    for (matrixHudIndicatorIndex = 0; matrixHudIndicatorIndex < NUM_MATRIX_HUD_INDICATORS;
         matrixHudIndicatorIndex++)
    {
        RF_CALL(matrixHudIndicatorDrawers[matrixHudIndicatorIndex].initialize());

        drivers->refSerial.sendGraphic(&matrixHudLabelAndTitleGraphics[matrixHudIndicatorIndex]);
        DELAY_REF_GRAPHIC(&matrixHudLabelAndTitleGraphics[matrixHudIndicatorIndex]);
    }

    drivers->refSerial.sendGraphic(&matrixHudLabelAndTitleGraphics[NUM_MATRIX_HUD_INDICATORS]);
    DELAY_REF_GRAPHIC(&matrixHudLabelAndTitleGraphics[NUM_MATRIX_HUD_INDICATORS]);

    RF_END();
}

modm::ResumableResult<bool> PositionHudIndicators::update()
{
    RF_BEGIN(1);

    updateIndicatorState();

    // draw all matrixHudIndicatorDrawers (only actually sends data if graphic changed)
    for (matrixHudIndicatorIndex = 0; matrixHudIndicatorIndex < NUM_MATRIX_HUD_INDICATORS;
         matrixHudIndicatorIndex++)
    {
        RF_CALL(matrixHudIndicatorDrawers[matrixHudIndicatorIndex].draw());
    }

    RF_END();
}

void PositionHudIndicators::updateIndicatorState()
{
    // update chassis state
    for (size_t i = 0; i < driveCommands.size(); i++)
    {
        if (drivers->commandScheduler.isCommandScheduled(driveCommands[i]))
        {
            currDriveCommandIndex = i;
        }
    }

    if (currDriveCommandIndex != -1)
    {
        matrixHudIndicatorDrawers[CHASSIS_STATE].setIndicatorState(
            MATRIX_HUD_INDICATOR_LABELS_START_Y -
            CHARACTER_LINE_SPACING * currDriveCommandIndex * MATRIX_HUD_INDICATOR_CHAR_SIZE -
            MATRIX_HUD_INDICATOR_CHAR_SIZE - MATRIX_HUD_INDICATOR_SELECTOR_BOX_WIDTH - 1);
    }

    // update flywheel and hopper state
    bool flywheelsOff =
        compareFloatClose(0.0f, frictionWheelSubsystem.getDesiredLaunchSpeed(), 1E-5);

    ShooterState shooterState =
        flywheelsOff ? ShooterState::FLYWHEELS_OFF : ShooterState::READY_TO_FIRE;

    if (shooterState == ShooterState::READY_TO_FIRE)
    {
#if defined(ALL_SOLDIERS)
        if (hopperSubsystem != nullptr && hopperSubsystem->getIsHopperOpen())
        {
            shooterState = ShooterState::LOADING;
        }
#elif defined(TARGET_HERO)
        if (drivers->turretMCBCanComm.getLimitSwitchDepressed())
        {
            shooterState = ShooterState::LOADING;
        }
#endif
    }

    matrixHudIndicatorDrawers[FLYWHEEL_AND_HOPPER_STATE].setIndicatorState(
        MATRIX_HUD_INDICATOR_LABELS_START_Y -
        CHARACTER_LINE_SPACING * static_cast<int>(shooterState) * MATRIX_HUD_INDICATOR_CHAR_SIZE -
        MATRIX_HUD_INDICATOR_CHAR_SIZE - MATRIX_HUD_INDICATOR_SELECTOR_BOX_WIDTH - 1);
}

void PositionHudIndicators::initialize()
{
    uint8_t matrixHudIndicatorName[3];
    getUnusedListName(matrixHudIndicatorName);

    uint16_t hudIndicatorListCurrX = MATRIX_HUD_INDICATOR_START_X;

    // Configure hopper cover hud indicator
    for (int i = 0; i < NUM_MATRIX_HUD_INDICATORS; i++)
    {
        // configure rectangle

        RefSerial::configGraphicGenerics(
            &matrixHudIndicatorGraphics[i].graphicData,
            matrixHudIndicatorName,
            Tx::ADD_GRAPHIC,
            DEFAULT_GRAPHIC_LAYER,
            MATRIX_HUD_INDICATOR_SELECTOR_BOX_COLOR);

        getUnusedListName(matrixHudIndicatorName);

        RefSerial::configRectangle(
            MATRIX_HUD_INDICATOR_SELECTOR_BOX_WIDTH,
            hudIndicatorListCurrX - 2 * MATRIX_HUD_INDICATOR_SELECTOR_BOX_WIDTH - 1,
            MATRIX_HUD_INDICATOR_LABELS_START_Y - MATRIX_HUD_INDICATOR_CHAR_SIZE -
                MATRIX_HUD_INDICATOR_SELECTOR_BOX_WIDTH - 1,
            hudIndicatorListCurrX +
                MATRIX_HUD_INDICATOR_TITLE_WIDTH * MATRIX_HUD_INDICATOR_CHAR_SIZE +
                MATRIX_HUD_INDICATOR_SELECTOR_BOX_WIDTH + 1,
            MATRIX_HUD_INDICATOR_LABELS_START_Y + MATRIX_HUD_INDICATOR_SELECTOR_BOX_WIDTH + 1,
            &matrixHudIndicatorGraphics[i].graphicData);

        // configure labels

        RefSerial::configGraphicGenerics(
            &matrixHudLabelAndTitleGraphics[i].graphicData,
            matrixHudIndicatorName,
            Tx::ADD_GRAPHIC,
            DEFAULT_GRAPHIC_LAYER,
            MATRIX_HUD_INDICATOR_LABELS_COLOR);

        getUnusedListName(matrixHudIndicatorName);

        RefSerial::configCharacterMsg(
            MATRIX_HUD_INDICATOR_CHAR_SIZE,
            MATRIX_HUD_INDICATOR_CHAR_LINE_WIDTH,
            hudIndicatorListCurrX,
            MATRIX_HUD_INDICATOR_LABELS_START_Y,
            MATRIX_HUD_INDICATOR_TITLES_AND_LABELS[i][1],
            &matrixHudLabelAndTitleGraphics[i]);

        hudIndicatorListCurrX += MATRIX_HUD_INDICATOR_TITLE_WIDTH * MATRIX_HUD_INDICATOR_CHAR_SIZE +
                                 MATRIX_HUD_INDICATOR_DIST_BTWN_INDICATOR_COLS;
    }

    // title

    RefSerial::configGraphicGenerics(
        &matrixHudLabelAndTitleGraphics[NUM_MATRIX_HUD_INDICATORS].graphicData,
        matrixHudIndicatorName,
        Tx::ADD_GRAPHIC,
        DEFAULT_GRAPHIC_LAYER,
        MATRIX_HUD_INDICATOR_TITLE_COLOR);

    char positionHudGraphicTitles[30];
    char *currHudGraphicTitlePos = positionHudGraphicTitles;

    const int spacesBetweenCols =
        MATRIX_HUD_INDICATOR_DIST_BTWN_INDICATOR_COLS / MATRIX_HUD_INDICATOR_CHAR_SIZE;

    // The individual titles are values in MATRIX_HUD_INDICATOR_TITLES_AND_LABELS, so copy over
    // these individual titles into a single character buffer to be sent in a graphic message
    for (int i = 0; i < NUM_MATRIX_HUD_INDICATORS; i++)
    {
        strcpy(currHudGraphicTitlePos, MATRIX_HUD_INDICATOR_TITLES_AND_LABELS[i][0]);
        currHudGraphicTitlePos += MATRIX_HUD_INDICATOR_TITLE_WIDTH;

        if (i != NUM_MATRIX_HUD_INDICATORS - 1)
        {
            // put spacesBetweenCols spaces between each title
            for (int j = 0; j < spacesBetweenCols; j++)
            {
                *currHudGraphicTitlePos = ' ';
                currHudGraphicTitlePos++;
            }
        }
    }

    RefSerial::configCharacterMsg(
        MATRIX_HUD_INDICATOR_CHAR_SIZE,
        MATRIX_HUD_INDICATOR_CHAR_LINE_WIDTH,
        MATRIX_HUD_INDICATOR_START_X,
        MATRIX_HUD_INDICATOR_TITLE_START_Y,
        positionHudGraphicTitles,
        &matrixHudLabelAndTitleGraphics[NUM_MATRIX_HUD_INDICATORS]);
}
}  // namespace aruwsrc::control::client_display