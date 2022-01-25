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

#include "client_display_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "aruwsrc/drivers.hpp"

#include "client_display_subsystem.hpp"

using namespace tap::control;
using namespace tap::serial;
using namespace tap::algorithms;
using namespace tap::communication::serial::ref_serial_ui_wrapeprs;
using namespace aruwsrc::control;
using namespace aruwsrc::control::launcher;
using namespace aruwsrc::agitator;

#define delay(graphic)                                                             \
    delayTimer.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(graphic)); \
    RF_WAIT_UNTIL(delayTimer.isExpired() || delayTimer.isStopped());

namespace aruwsrc::display
{
/**
 * Updates the color of a graphic to be either `ON_COLOR` if `indicatorStatus == true` or
 * `OFF_COLOR` if `indicatorStatus == false`.
 */
template <
    tap::serial::RefSerial::Tx::GraphicColor ON_COLOR,
    tap::serial::RefSerial::Tx::GraphicColor OFF_COLOR>
static void updateGraphicColor(
    bool indicatorStatus,
    tap::serial::RefSerialData::Tx::Graphic1Message *graphic)
{
    graphic->graphicData.color =
        static_cast<uint32_t>(indicatorStatus ? ON_COLOR : OFF_COLOR) & 0xf;
}

/**
 * Update the start and end Y coordinates of a graphic based on the specified `location`.
 *
 * @note By convention expected that `endY > startY`. If this is not true, this function does
 * nothing.
 */
static void updateGraphicYLocation(
    uint16_t location,
    tap::serial::RefSerialData::Tx::Graphic1Message *graphic)
{
    if (graphic->graphicData.endY < graphic->graphicData.startY)
    {
        return;
    }

    uint16_t startYDiff = graphic->graphicData.endY - graphic->graphicData.startY;
    graphic->graphicData.startY = location;
    graphic->graphicData.endY = location + startYDiff;
}

ClientDisplayCommand::ClientDisplayCommand(
    aruwsrc::Drivers *drivers,
    ClientDisplaySubsystem *clientDisplay,
    const TurretMCBHopperSubsystem *hopperSubsystem,
    const FrictionWheelSubsystem *frictionWheelSubsystem,
    AgitatorSubsystem *agitatorSubsystem,
    const aruwsrc::control::turret::TurretSubsystem *turretSubsystem,
    const aruwsrc::chassis::BeybladeCommand *chassisBeybladeCmd,
    const aruwsrc::chassis::ChassisAutorotateCommand *chassisAutorotateCmd,
    const aruwsrc::chassis::ChassisImuDriveCommand *chassisImuDriveCommand,
    const aruwsrc::chassis::ChassisDriveCommand *chassisDriveCmd)
    : Command(),
      drivers(drivers),
      hopperSubsystem(hopperSubsystem),
      frictionWheelSubsystem(frictionWheelSubsystem),
      agitatorSubsystem(agitatorSubsystem),
      booleanHudIndicatorDrawers{
          BooleanHUDIndicator(
              drivers,
              &booleanHudIndicatorGraphics[SYSTEMS_CALIBRATING],
              updateGraphicColor<
                  std::get<1>(BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[SYSTEMS_CALIBRATING]),
                  std::get<2>(BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[SYSTEMS_CALIBRATING])>),
          BooleanHUDIndicator(
              drivers,
              &booleanHudIndicatorGraphics[AGITATOR_STATUS_HEALTHY],
              updateGraphicColor<
                  std::get<1>(BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[AGITATOR_STATUS_HEALTHY]),
                  std::get<2>(BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[AGITATOR_STATUS_HEALTHY])>),
          BooleanHUDIndicator(
              drivers,
              &booleanHudIndicatorGraphics[CV_AIM_DATA_VALID],
              updateGraphicColor<
                  std::get<1>(BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[CV_AIM_DATA_VALID]),
                  std::get<2>(BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[CV_AIM_DATA_VALID])>),
      },
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
              updateGraphicYLocation),
          StateHUDIndicator<uint16_t>(
              drivers,
              &matrixHudIndicatorGraphics[FLYWHEEL_AND_HOPPER_STATE],
              updateGraphicYLocation),
          StateHUDIndicator<uint16_t>(
              drivers,
              &matrixHudIndicatorGraphics[SHOOTER_STATE],
              updateGraphicYLocation)},
      turretSubsystem(turretSubsystem)
{
    modm_assert(
        drivers != nullptr && clientDisplay != nullptr && hopperSubsystem != nullptr &&
            frictionWheelSubsystem != nullptr && agitatorSubsystem != nullptr &&
            turretSubsystem != nullptr,
        "ClientDisplayCommand",
        "some arguments not allowed to be nullptr are nullptr");
    addSubsystemRequirement(clientDisplay);
}

void ClientDisplayCommand::initialize()
{
    resetListNameGenerator();
    restart();  // restart protothread
    initializeBooleanHudIndicators();
    initializePositionHudIndicators();
    initializeReticle();
    initializeChassisOrientation();
    initializeTurretAngles();
}

void ClientDisplayCommand::execute() { run(); }

bool ClientDisplayCommand::run()
{
    PT_BEGIN();

    PT_CALL(sendInitialGraphics());

    while (true)
    {
        PT_CALL(updateBooleanHudIndicators());
        PT_CALL(updatePositionSelectorHudIndicators());
        PT_CALL(updateChassisOrientation());
        PT_CALL(updateTurretAngles());
        PT_YIELD();
    }
    PT_END();
}

modm::ResumableResult<bool> ClientDisplayCommand::sendInitialGraphics()
{
    RF_BEGIN(0);

    RF_WAIT_UNTIL(drivers->refSerial.getRefSerialReceivingData());

    // send all boolean hud indicator graphics (labels and circles)
    for (booleanHudIndicatorIndex = 0; booleanHudIndicatorIndex < NUM_BOOLEAN_HUD_INDICATORS;
         booleanHudIndicatorIndex++)
    {
        RF_CALL(booleanHudIndicatorDrawers[booleanHudIndicatorIndex].initialize());

        drivers->refSerial.sendGraphic(
            &booleanHudIndicatorStaticGrahpics[booleanHudIndicatorIndex]);
        delay(&booleanHudIndicatorStaticGrahpics[booleanHudIndicatorIndex]);
        drivers->refSerial.sendGraphic(
            &booleanHudIndicatorStaticLabelGraphics[booleanHudIndicatorIndex]);
        delay(&booleanHudIndicatorStaticLabelGraphics[booleanHudIndicatorIndex]);
    }

    // send all matrix HUD indicator-related graphics (the labels, title, and boxes)
    for (matrixHudIndicatorIndex = 0; matrixHudIndicatorIndex < NUM_MATRIX_HUD_INDICATORS;
         matrixHudIndicatorIndex++)
    {
        RF_CALL(matrixHudIndicatorDrawers[matrixHudIndicatorIndex].initialize());

        drivers->refSerial.sendGraphic(&matrixHudLabelAndTitleGraphics[matrixHudIndicatorIndex]);
        delay(&matrixHudLabelAndTitleGraphics[matrixHudIndicatorIndex]);
    }

    drivers->refSerial.sendGraphic(&matrixHudLabelAndTitleGraphics[NUM_MATRIX_HUD_INDICATORS]);
    delay(&matrixHudLabelAndTitleGraphics[NUM_MATRIX_HUD_INDICATORS]);

    // send reticle
    for (reticleIndex = 0; reticleIndex < MODM_ARRAY_SIZE(reticleMsg); reticleIndex++)
    {
        drivers->refSerial.sendGraphic(&reticleMsg[reticleIndex]);
        delay(&reticleMsg[reticleIndex]);
    }

    // send initial chassis orientation graphics
    drivers->refSerial.sendGraphic(&chassisOrientationGraphics);
    delay(&chassisOrientationGraphics);
    chassisOrientationGraphics.graphicData[0].operation = Tx::ADD_GRAPHIC_MODIFY;
    chassisOrientationGraphics.graphicData[1].operation = Tx::ADD_GRAPHIC_MODIFY;

    // send turret angle data graphic and associated labels
    drivers->refSerial.sendGraphic(&turretAnglesGraphics);
    delay(&turretAnglesGraphics);
    turretAnglesGraphics.graphicData.operation = Tx::ADD_GRAPHIC_MODIFY;

    drivers->refSerial.sendGraphic(&turretAnglesLabelGraphics);
    delay(&turretAnglesLabelGraphics);

    RF_END();
}

modm::ResumableResult<bool> ClientDisplayCommand::updateBooleanHudIndicators()
{
    RF_BEGIN(1);

    // update CV aim data state
    booleanHudIndicatorDrawers[CV_AIM_DATA_VALID].setIndicatorState(
        drivers->legacyVisionCoprocessor.lastAimDataValid() &&
        drivers->legacyVisionCoprocessor.getLastAimData().hasTarget);

    // update agitator state
    booleanHudIndicatorDrawers[AGITATOR_STATUS_HEALTHY].setIndicatorState(
        agitatorSubsystem->isOnline() && !agitatorSubsystem->isJammed());

    // TODO add hero fire ready and systems calibrating when available
    booleanHudIndicatorDrawers[SYSTEMS_CALIBRATING].setIndicatorState(true);

    // draw all the booleanHudIndicatorDrawers (only actually sends data if graphic changed)
    for (booleanHudIndicatorIndex = 0; booleanHudIndicatorIndex < NUM_BOOLEAN_HUD_INDICATORS;
         booleanHudIndicatorIndex++)
    {
        RF_CALL(booleanHudIndicatorDrawers[booleanHudIndicatorIndex].draw());
    }

    RF_END();
}

modm::ResumableResult<bool> ClientDisplayCommand::updatePositionSelectorHudIndicators()
{
    RF_BEGIN(2);

    updatePositionSelectionHudIndicatorState();

    // draw all matrixHudIndicatorDrawers (only actually sends data if graphic changed)
    for (matrixHudIndicatorIndex = 0; matrixHudIndicatorIndex < NUM_MATRIX_HUD_INDICATORS;
         matrixHudIndicatorIndex++)
    {
        RF_CALL(matrixHudIndicatorDrawers[matrixHudIndicatorIndex].draw());
    }

    RF_END();
}

void ClientDisplayCommand::updatePositionSelectionHudIndicatorState()
{
    // update chassis state
    for (size_t i = 0; i < driveCommands.size(); i++)
    {
        if (drivers->commandScheduler.isCommandScheduled(driveCommands[i]))
        {
            currDriveCommandIndex = i;
        }
    }
    matrixHudIndicatorDrawers[CHASSIS_STATE].setIndicatorState(
        MATRIX_HUD_INDICATOR_LABELS_START_Y -
        CHARACTER_LINE_SPACING * currDriveCommandIndex * MATRIX_HUD_INDICATOR_CHAR_SIZE -
        MATRIX_HUD_INDICATOR_CHAR_SIZE - MATRIX_HUD_INDICATOR_SELECTOR_BOX_WIDTH - 1);

    // update flywheel and hopper state
    bool flywheelsOff =
        compareFloatClose(0.0f, frictionWheelSubsystem->getDesiredLaunchSpeed(), 1E-5);
    bool hopperOpen = hopperSubsystem->getIsHopperOpen();

    int hopperIndex;
    if (!flywheelsOff && !hopperOpen)
    {
        // if flywheels on and hopper closed index 0
        hopperIndex = 0;
    }
    if (!flywheelsOff && hopperOpen)
    {
        // if flywheels on and hopper open index 1
        hopperIndex = 1;
    }
    else if (flywheelsOff)
    {
        // if flywheels off index 2
        hopperIndex = 2;
    }
    matrixHudIndicatorDrawers[FLYWHEEL_AND_HOPPER_STATE].setIndicatorState(
        MATRIX_HUD_INDICATOR_LABELS_START_Y -
        CHARACTER_LINE_SPACING * hopperIndex * MATRIX_HUD_INDICATOR_CHAR_SIZE -
        MATRIX_HUD_INDICATOR_CHAR_SIZE - MATRIX_HUD_INDICATOR_SELECTOR_BOX_WIDTH - 1);

    // TODO update firing state
}

modm::ResumableResult<bool> ClientDisplayCommand::updateChassisOrientation()
{
    RF_BEGIN(3);

    // update chassisOrientation if turret is online, otherwise don't rotate
    // chassis
    chassisOrientation.rotate(modm::toRadian(
        (turretSubsystem->isOnline()) ? -turretSubsystem->getYawAngleFromCenter() : 0.0f));

    // if chassis orientation has changed, send new graphic with updated orientation
    if (chassisOrientation != chassisOrientationPrev)
    {
        // since chassisOrientation is a pixel coordinate centered around
        // `CHASSIS_CENTER_X/Y`, when configuring the line center it about these coordinates
        RefSerial::configLine(
            CHASSIS_WIDTH,
            CHASSIS_CENTER_X + chassisOrientation.x,
            CHASSIS_CENTER_Y + chassisOrientation.y,
            CHASSIS_CENTER_X - chassisOrientation.x,
            CHASSIS_CENTER_Y - chassisOrientation.y,
            &chassisOrientationGraphics.graphicData[0]);
        drivers->refSerial.sendGraphic(&chassisOrientationGraphics);
        delay(&chassisOrientationGraphics);

        chassisOrientationPrev = chassisOrientation;
    }

    // reset rotated orientation back to forward orientation so next time chassisOrientation
    // is rotated by `getYawAngleFromCenter` the rotation is relative to the forward.
    chassisOrientation.set(0, CHASSIS_LENGTH / 2);

    RF_END();
}

modm::ResumableResult<bool> ClientDisplayCommand::updateTurretAngles()
{
    RF_BEGIN(4);

    yaw = drivers->turretMCBCanComm.isConnected() ? drivers->turretMCBCanComm.getYaw() : 0.0f;
#if defined(TARGET_HERO)
    pitch = turretSubsystem->getCurrentPitchValue().getValue();
#else
    pitch = drivers->turretMCBCanComm.isConnected() ? drivers->turretMCBCanComm.getPitch() : 0.0f;
#endif

    if (sendTurretDataTimer.execute() &&
        (!compareFloatClose(prevYaw, yaw, 1.0f / TURRET_ANGLES_DECIMAL_PRECISION) ||
         !compareFloatClose(prevPitch, pitch, 1.0f / TURRET_ANGLES_DECIMAL_PRECISION)))
    {
        // set the character buffer `turretAnglesGraphics.msg` to the turret pitch/yaw angle
        // values
        // note that `%f` doesn't work in `sprintf` and neither do the integer or floating point
        // graphics, so this is why we are using `sprintf` to put floating point numbers in a
        // character graphic
        bytesWritten = sprintf(
            turretAnglesGraphics.msg,
            "%i.%i\n\n%i.%i",
            static_cast<int>(yaw),
            abs(static_cast<int>(yaw * TURRET_ANGLES_DECIMAL_PRECISION) %
                TURRET_ANGLES_DECIMAL_PRECISION),
            static_cast<int>(pitch),
            abs(static_cast<int>(pitch * TURRET_ANGLES_DECIMAL_PRECISION) %
                TURRET_ANGLES_DECIMAL_PRECISION));
        // `endAngle` is actually length of the string
        turretAnglesGraphics.graphicData.endAngle = bytesWritten;

        drivers->refSerial.sendGraphic(&turretAnglesGraphics);
        delay(&turretAnglesGraphics);

        prevYaw = yaw;
        prevPitch = pitch;
    }

    RF_END();
}

void ClientDisplayCommand::initializeBooleanHudIndicators()
{
    uint8_t booleanHudIndicatorName[3] = {};
    uint16_t hudIndicatorListCurrY = BOOLEAN_HUD_INDICATOR_LIST_START_Y;

    // Configure hopper cover hud indicator
    for (int i = 0; i < NUM_BOOLEAN_HUD_INDICATORS; i++)
    {
        // config the boolean HUD indicator graphic
        getUnusedListName(booleanHudIndicatorName);

        RefSerial::configGraphicGenerics(
            &booleanHudIndicatorGraphics[i].graphicData,
            booleanHudIndicatorName,
            Tx::ADD_GRAPHIC,
            DEFAULT_GRAPHIC_LAYER,
            std::get<2>(BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[i]));

        RefSerial::configCircle(
            BOOLEAN_HUD_INDICATOR_WIDTH,
            BOOLEAN_HUD_INDICATOR_LIST_CENTER_X,
            hudIndicatorListCurrY,
            BOOLEAN_HUD_INDICATOR_RADIUS,
            &booleanHudIndicatorGraphics[i].graphicData);

        // config the border circle that bounds the booleanHudIndicatorGraphics
        getUnusedListName(booleanHudIndicatorName);

        RefSerial::configGraphicGenerics(
            &booleanHudIndicatorStaticGrahpics[i].graphicData,
            booleanHudIndicatorName,
            Tx::ADD_GRAPHIC,
            DEFAULT_GRAPHIC_LAYER,
            BOOLEAN_HUD_INDICATOR_OUTLINE_COLOR);

        RefSerial::configCircle(
            BOOLEAN_HUD_INDICATOR_OUTLINE_WIDTH,
            BOOLEAN_HUD_INDICATOR_LIST_CENTER_X,
            hudIndicatorListCurrY,
            BOOLEAN_HUD_INDICATOR_OUTLINE_RADIUS,
            &booleanHudIndicatorStaticGrahpics[i].graphicData);

        // config the label associated with the particular indicator
        getUnusedListName(booleanHudIndicatorName);

        RefSerial::configGraphicGenerics(
            &booleanHudIndicatorStaticLabelGraphics[i].graphicData,
            booleanHudIndicatorName,
            Tx::ADD_GRAPHIC,
            DEFAULT_GRAPHIC_LAYER,
            BOOLEAN_HUD_INDICATOR_LABEL_COLOR);

        const char *indicatorLabel = std::get<0>(BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[i]);

        RefSerial::configCharacterMsg(
            BOOLEAN_HUD_INDICATOR_LABEL_CHAR_SIZE,
            BOOLEAN_HUD_INDICATOR_LABEL_CHAR_LINE_WIDTH,
            BOOLEAN_HUD_INDICATOR_LIST_CENTER_X -
                strlen(indicatorLabel) * BOOLEAN_HUD_INDICATOR_LABEL_CHAR_SIZE -
                BOOLEAN_HUD_INDICATOR_OUTLINE_RADIUS - BOOLEAN_HUD_INDICATOR_OUTLINE_WIDTH / 2,
            hudIndicatorListCurrY + BOOLEAN_HUD_INDICATOR_LABEL_CHAR_SIZE / 2,
            indicatorLabel,
            &booleanHudIndicatorStaticLabelGraphics[i]);

        // shift the Y pixel location down so the next indicator will be below the indicator was
        // just configured
        hudIndicatorListCurrY -= BOOLEAN_HUD_INDICATOR_LIST_DIST_BTWN_BULLETS;
    }
}
void ClientDisplayCommand::initializePositionHudIndicators()
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

void ClientDisplayCommand::initializeReticle()
{
    uint8_t currLineName[3];
    getUnusedListName(currLineName);

    // Add reticle markers
    uint16_t maxReticleY = 0;
    uint16_t minReticleY = UINT16_MAX;

    // configure all horizontal reticle lines
    for (size_t i = 0; i < NUM_RETICLE_COORDINATES; i++)
    {
        // reticleMsg is an array of Graphic5Messages, so find the index in the array and in the
        // individual Graphic5Message
        size_t reticleMsgIndex = i / MODM_ARRAY_SIZE(reticleMsg[0].graphicData);
        size_t graphicDataIndex = i % MODM_ARRAY_SIZE(reticleMsg[0].graphicData);

        RefSerial::configGraphicGenerics(
            &reticleMsg[reticleMsgIndex].graphicData[graphicDataIndex],
            currLineName,
            Tx::ADD_GRAPHIC,
            DEFAULT_GRAPHIC_LAYER,
            std::get<2>(TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES[i]));

        getUnusedListName(currLineName);

        // center of the reticle, in pixels
        uint16_t reticleXCenter = static_cast<int>(SCREEN_WIDTH / 2) + RETICLE_CENTER_X_OFFSET;

        // start and end X pixel coordinates of the current reticle line
        uint16_t startX =
            reticleXCenter - std::get<0>(TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES[i]);
        uint16_t endX =
            reticleXCenter + std::get<0>(TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES[i]);

        // y coordinate of the horizontal reticle line
        uint16_t y = std::get<1>(TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES[i]);

        RefSerial::configLine(
            RETICLE_THICKNESS,
            startX,
            y,
            endX,
            y,
            &reticleMsg[reticleMsgIndex].graphicData[graphicDataIndex]);

        // update min and max y coordinates to be used when drawing the vertical reticle line that
        // connects horizontal reticle lines
        if (y > maxReticleY)
        {
            maxReticleY = y;
        }
        if (y < minReticleY)
        {
            minReticleY = y;
        }
    }

    // Add horizontal reticle line to connect reticle markers
    RefSerial::configGraphicGenerics(
        &reticleMsg[NUM_RETICLE_COORDINATES / MODM_ARRAY_SIZE(reticleMsg[0].graphicData)]
             .graphicData[NUM_RETICLE_COORDINATES % MODM_ARRAY_SIZE(reticleMsg[0].graphicData)],
        currLineName,
        Tx::ADD_GRAPHIC,
        DEFAULT_GRAPHIC_LAYER,
        RETICLE_VERTICAL_COLOR);

    RefSerial::configLine(
        RETICLE_THICKNESS,
        SCREEN_WIDTH / 2 + RETICLE_CENTER_X_OFFSET,
        minReticleY,
        SCREEN_WIDTH / 2 + RETICLE_CENTER_X_OFFSET,
        maxReticleY,
        &reticleMsg[NUM_RETICLE_COORDINATES / MODM_ARRAY_SIZE(reticleMsg[0].graphicData)]
             .graphicData[NUM_RETICLE_COORDINATES % MODM_ARRAY_SIZE(reticleMsg[0].graphicData)]);
}

void ClientDisplayCommand::initializeChassisOrientation()
{
    // chassis orientation starts forward facing
    chassisOrientation.set(0, CHASSIS_LENGTH / 2);
    chassisOrientationPrev = chassisOrientation;

    uint8_t chassisOrientationName[3];
    getUnusedListName(chassisOrientationName);

    // config the chassis graphic

    RefSerial::configGraphicGenerics(
        &chassisOrientationGraphics.graphicData[0],
        chassisOrientationName,
        Tx::ADD_GRAPHIC,
        DEFAULT_GRAPHIC_LAYER,
        CHASSIS_ORIENTATION_COLOR);

    RefSerial::configLine(
        CHASSIS_WIDTH,
        CHASSIS_CENTER_X + chassisOrientation.x,
        CHASSIS_CENTER_Y + chassisOrientation.y,
        CHASSIS_CENTER_X - chassisOrientation.x,
        CHASSIS_CENTER_Y - chassisOrientation.y,
        &chassisOrientationGraphics.graphicData[0]);

    getUnusedListName(chassisOrientationName);

    // config the turret graphic

    RefSerial::configGraphicGenerics(
        &chassisOrientationGraphics.graphicData[1],
        chassisOrientationName,
        Tx::ADD_GRAPHIC,
        DEFAULT_GRAPHIC_LAYER,
        CHASSIS_BARREL_COLOR);

    RefSerial::configLine(
        CHASSIS_BARREL_WIDTH,
        CHASSIS_CENTER_X,
        CHASSIS_CENTER_Y,
        CHASSIS_CENTER_X,
        CHASSIS_CENTER_Y + CHASSIS_BARREL_LENGTH,
        &chassisOrientationGraphics.graphicData[1]);
}

void ClientDisplayCommand::initializeTurretAngles()
{
    uint8_t turretAnglesName[3];
    getUnusedListName(turretAnglesName);

    RefSerial::configGraphicGenerics(
        &turretAnglesGraphics.graphicData,
        turretAnglesName,
        Tx::ADD_GRAPHIC,
        DEFAULT_GRAPHIC_LAYER,
        TURRET_ANGLES_COLOR);

    RefSerial::configCharacterMsg(
        TURRET_ANGLES_CHAR_SIZE,
        TURRET_ANGLES_CHAR_WIDTH,
        TURRET_ANGLES_START_X,
        TURRET_ANGLES_START_Y,
        "0\n\n0",
        &turretAnglesGraphics);

    turretAnglesName[2]++;

    RefSerial::configGraphicGenerics(
        &turretAnglesLabelGraphics.graphicData,
        turretAnglesName,
        Tx::ADD_GRAPHIC,
        DEFAULT_GRAPHIC_LAYER,
        TURRET_ANGLES_COLOR);

    RefSerial::configCharacterMsg(
        TURRET_ANGLES_CHAR_SIZE,
        TURRET_ANGLES_CHAR_WIDTH,
        TURRET_ANGLES_START_X - strlen("PITCH: ") * TURRET_ANGLES_CHAR_SIZE,
        TURRET_ANGLES_START_Y,
        "  YAW:\n\nPITCH:",
        &turretAnglesLabelGraphics);

    prevYaw = 0.0f;
    prevPitch = 0.0f;
}

void ClientDisplayCommand::resetListNameGenerator() { currListName = 0; }

void ClientDisplayCommand::getUnusedListName(uint8_t listName[3])
{
    if (currListName > 0xffffff)
    {
        RAISE_ERROR(drivers, "all graphic names used");
    }
    else
    {
        listName[0] = static_cast<uint8_t>((currListName >> 16) & 0xff);
        listName[1] = static_cast<uint8_t>((currListName >> 8) & 0xff);
        listName[2] = static_cast<uint8_t>(currListName & 0xff);
        currListName++;
    }
}

}  // namespace aruwsrc::display
