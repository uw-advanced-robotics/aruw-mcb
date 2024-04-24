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

#include "matrix_hud_indicators.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/util_macros.hpp"

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
 * @note This function asserts that `endY > startY` must be true (as it indicates a bug otherwise).
 */
static inline void updateGraphicYLocation(
    uint16_t location,
    RefSerialData::Tx::Graphic1Message *graphic)
{
    modm_assert(
        graphic->graphicData.endY >= graphic->graphicData.startY,
        "updateGraphicYLocation",
        "invalid graphic data struct");

    uint16_t startYDiff = graphic->graphicData.endY - graphic->graphicData.startY;
    graphic->graphicData.startY = location;
    graphic->graphicData.endY = location + startYDiff;
}

MatrixHudIndicators::MatrixHudIndicators(
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
    const aruwsrc::chassis::ChassisImuDriveCommand *chassisImuDriveCommand)
    : HudIndicator(refSerialTransmitter),
      drivers(drivers),
      visionCoprocessor(visionCoprocessor),
      hopperSubsystem(hopperSubsystem),
      frictionWheelSubsystem(frictionWheelSubsystem),
      turretSubsystem(turretSubsystem),
      multiShotHandler(multiShotHandler),
      cvOnTargetGovernor(cvOnTargetGovernor),
      driveCommands{
          chassisBeybladeCmd,
          chassisAutorotateCmd,
          chassisImuDriveCommand,
      },
      matrixHudIndicatorDrawers
{
    StateHUDIndicator<uint16_t>(
        refSerialTransmitter,
        &matrixHudIndicatorGraphics[CHASSIS_STATE],
        updateGraphicYLocation,
        0),
        StateHUDIndicator<uint16_t>(
            refSerialTransmitter,
            &matrixHudIndicatorGraphics[SHOOTER_STATE],
            updateGraphicYLocation,
            0),
#if defined(DISPLAY_FIRING_MODE)
        StateHUDIndicator<uint16_t>(
            refSerialTransmitter,
            &matrixHudIndicatorGraphics[FIRING_MODE],
            updateGraphicYLocation,
            0),
#endif
        StateHUDIndicator<uint16_t>(
            refSerialTransmitter,
            &matrixHudIndicatorGraphics[CV_STATUS],
            updateGraphicYLocation,
            0),
}
{
}

modm::ResumableResult<bool> MatrixHudIndicators::sendInitialGraphics()
{
    RF_BEGIN(0);

    // send all matrix HUD indicator-related graphics (the labels, title, and boxes)
    for (matrixHudIndicatorIndex = 0; matrixHudIndicatorIndex < NUM_MATRIX_HUD_INDICATORS;
         matrixHudIndicatorIndex++)
    {
        RF_CALL(matrixHudIndicatorDrawers[matrixHudIndicatorIndex].initialize());

        RF_CALL(refSerialTransmitter.sendGraphic(
            &matrixHudLabelAndTitleGraphics[matrixHudIndicatorIndex]));
    }

    RF_CALL(refSerialTransmitter.sendGraphic(
        &matrixHudLabelAndTitleGraphics[NUM_MATRIX_HUD_INDICATORS]));

    RF_END();
}

modm::ResumableResult<bool> MatrixHudIndicators::update()
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

void MatrixHudIndicators::updateIndicatorState()
{
    // update chassis state
    for (size_t i = 0; i < driveCommands.size(); i++)
    {
        if (drivers.commandScheduler.isCommandScheduled(driveCommands[i]))
        {
            currDriveCommandIndex = i;
        }
    }

    if (currDriveCommandIndex != -1)
    {
        matrixHudIndicatorDrawers[CHASSIS_STATE].setIndicatorState(
            getIndicatorYCoordinate(currDriveCommandIndex));
    }

    // update flywheel and hopper state
    bool flywheelsOff =
        compareFloatClose(0.0f, frictionWheelSubsystem.getDesiredLaunchSpeed(), 1E-5);

    ShooterState shooterState =
        flywheelsOff ? ShooterState::FLYWHEELS_OFF : ShooterState::READY_TO_FIRE;

    if (shooterState == ShooterState::READY_TO_FIRE)
    {
#if defined(ALL_STANDARDS)
        if (hopperSubsystem != nullptr && hopperSubsystem->getIsHopperOpen())
        {
            shooterState = ShooterState::LOADING;
        }
#elif defined(TARGET_HERO_KRONOS)
        auto turretMCB = turretSubsystem.getTurretMCB();
        assert(turretMCB != nullptr);
        if (!turretMCB->getLimitSwitchDepressed())
        {
            shooterState = ShooterState::LOADING;
        }
#endif
    }

    matrixHudIndicatorDrawers[SHOOTER_STATE].setIndicatorState(
        getIndicatorYCoordinate(static_cast<int>(shooterState)));

#if defined(DISPLAY_FIRING_MODE)
    // update firing mode
    matrixHudIndicatorDrawers[FIRING_MODE].setIndicatorState(getIndicatorYCoordinate(
        static_cast<int>(multiShotHandler == nullptr ? 0 : multiShotHandler->getLaunchMode())));
#endif

    CVStatus cvStatus = CVStatus::VISION_COPROCESSOR_OFFLINE;

    if (visionCoprocessor.isCvOnline())
    {
        if (cvOnTargetGovernor == nullptr)
        {
            cvStatus = CVStatus::VISION_COPROCESSOR_NO_PROJECTILE_GATING;
        }
        else
        {
            cvStatus = cvOnTargetGovernor->isGoverEnabled()
                           ? CVStatus::VISION_COPROCESSOR_GATED_PROJECTILE_LAUNCH
                           : CVStatus::VISION_COPROCESSOR_NO_PROJECTILE_GATING;
        }
    }

    matrixHudIndicatorDrawers[CV_STATUS].setIndicatorState(
        getIndicatorYCoordinate(static_cast<int>(cvStatus)));
}

void MatrixHudIndicators::initialize()
{
    uint8_t matrixHudIndicatorName[3];
    getUnusedGraphicName(matrixHudIndicatorName);

    uint16_t hudIndicatorListCurrX = MATRIX_HUD_INDICATOR_START_X;

    // Configure hopper cover hud indicator
    for (int i = 0; i < NUM_MATRIX_HUD_INDICATORS; i++)
    {
        // configure rectangle

        RefSerialTransmitter::configGraphicGenerics(
            &matrixHudIndicatorGraphics[i].graphicData,
            matrixHudIndicatorName,
            Tx::GRAPHIC_ADD,
            DEFAULT_GRAPHIC_LAYER,
            MATRIX_HUD_INDICATOR_SELECTOR_BOX_COLOR);

        getUnusedGraphicName(matrixHudIndicatorName);

        RefSerialTransmitter::configRectangle(
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

        RefSerialTransmitter::configGraphicGenerics(
            &matrixHudLabelAndTitleGraphics[i].graphicData,
            matrixHudIndicatorName,
            Tx::GRAPHIC_ADD,
            DEFAULT_GRAPHIC_LAYER,
            MATRIX_HUD_INDICATOR_LABELS_COLOR);

        getUnusedGraphicName(matrixHudIndicatorName);

        RefSerialTransmitter::configCharacterMsg(
            MATRIX_HUD_INDICATOR_CHAR_SIZE,
            MATRIX_HUD_INDICATOR_CHAR_LINE_WIDTH,
            hudIndicatorListCurrX,
            MATRIX_HUD_INDICATOR_LABELS_START_Y,
            MATRIX_HUD_INDICATOR_TITLES_AND_LABELS[i][1],
            &matrixHudLabelAndTitleGraphics[i]);

        hudIndicatorListCurrX += MATRIX_HUD_INDICATOR_TITLE_WIDTH * MATRIX_HUD_INDICATOR_CHAR_SIZE +
                                 MATRIX_HUD_INDICATOR_DIST_BTWN_INDICATOR_COLS;
    }

    // configure the title (the first row of the matrix indicator, giving meaning to each labeled
    // column)

    RefSerialTransmitter::configGraphicGenerics(
        &matrixHudLabelAndTitleGraphics[NUM_MATRIX_HUD_INDICATORS].graphicData,
        matrixHudIndicatorName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        MATRIX_HUD_INDICATOR_TITLE_COLOR);

    char positionHudGraphicTitles[30];
    char *currHudGraphicTitlePos = positionHudGraphicTitles;

    static constexpr int spacesBetweenCols =
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

    RefSerialTransmitter::configCharacterMsg(
        MATRIX_HUD_INDICATOR_CHAR_SIZE,
        MATRIX_HUD_INDICATOR_CHAR_LINE_WIDTH,
        MATRIX_HUD_INDICATOR_START_X,
        MATRIX_HUD_INDICATOR_TITLE_START_Y,
        positionHudGraphicTitles,
        &matrixHudLabelAndTitleGraphics[NUM_MATRIX_HUD_INDICATORS]);
}
}  // namespace aruwsrc::control::client_display
