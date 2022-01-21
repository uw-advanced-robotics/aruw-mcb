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

static void updateGraphicYLocation(
    uint16_t location,
    tap::serial::RefSerialData::Tx::Graphic1Message *graphic)
{
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
#if defined(TARGET_HERO)
      booleanHudIndicatorDrawers{
          BooleanDrawer(
              drivers,
              &booleanHudIndicatorGraphics[0],
              updateGraphicColor<Tx::GraphicColor::YELLOW, Tx::GraphicColor::GREEN>),
          BooleanDrawer(
              drivers,
              &booleanHudIndicatorGraphics[1],
              updateGraphicColor<Tx::GraphicColor::GREEN, Tx::GraphicColor::RED_AND_BLUE>),
          BooleanDrawer(
              drivers,
              &booleanHudIndicatorGraphics[2],
              updateGraphicColor<Tx::GraphicColor::GREEN, Tx::GraphicColor::RED_AND_BLUE>),
          BooleanDrawer(
              drivers,
              &booleanHudIndicatorGraphics[3],
              updateGraphicColor<Tx::GraphicColor::GREEN, Tx::GraphicColor::RED_AND_BLUE>),
          BooleanDrawer(
              drivers,
              &booleanHudIndicatorGraphics[4],
              updateGraphicColor<Tx::GraphicColor::GREEN, Tx::GraphicColor::RED_AND_BLUE>),
          BooleanDrawer(
              drivers,
              &booleanHudIndicatorGraphics[5],
              updateGraphicColor<Tx::GraphicColor::GREEN, Tx::GraphicColor::RED_AND_BLUE>),
      },
#else
      booleanHudIndicatorDrawers{
          BooleanHUDIndicator(
              drivers,
              &booleanHudIndicatorGraphics[SYSTEMS_CALIBRATING],
              updateGraphicColor<Tx::GraphicColor::YELLOW, Tx::GraphicColor::GREEN>),
          BooleanHUDIndicator(
              drivers,
              &booleanHudIndicatorGraphics[CV_AIM_DATA_VALID],
              updateGraphicColor<Tx::GraphicColor::GREEN, Tx::GraphicColor::RED_AND_BLUE>),
          BooleanHUDIndicator(
              drivers,
              &booleanHudIndicatorGraphics[AGITATOR_STATUS_HEALTHY],
              updateGraphicColor<Tx::GraphicColor::GREEN, Tx::GraphicColor::RED_AND_BLUE>),
      },
#endif
      positionSelectionHudIndicatorDrawers{
          StateHUDIndicator<uint16_t>(
              drivers,
              &positionSelectionHudIndicatorGraphics[CHASSIS_STATE],
              updateGraphicYLocation),
          StateHUDIndicator<uint16_t>(
              drivers,
              &positionSelectionHudIndicatorGraphics[FLYWHEEL_AND_HOPPER_STATE],
              updateGraphicYLocation),
          StateHUDIndicator<uint16_t>(
              drivers,
              &positionSelectionHudIndicatorGraphics[SHOOTER_STATE],
              updateGraphicYLocation)},
      driveCommands{
          chassisBeybladeCmd,
          chassisAutorotateCmd,
          chassisImuDriveCommand,
          chassisDriveCmd,
      },
      turretSubsystem(turretSubsystem)
{
    addSubsystemRequirement(clientDisplay);
}

void ClientDisplayCommand::initialize()
{
    restart();  // restart protothread
    initializeBooleanHudIndicators();
    initializePositionHudIndicators();
    initializeReticle();
    // initializeDriveCommand();
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

        // PT_CALL(updateDriveCommandMsg());

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

    for (positionSelectionHudIndicatorIndex = 0;
         positionSelectionHudIndicatorIndex < NUM_POSITION_HUD_INDICATORS;
         positionSelectionHudIndicatorIndex++)
    {
        RF_CALL(
            positionSelectionHudIndicatorDrawers[positionSelectionHudIndicatorIndex].initialize());

        drivers->refSerial.sendGraphic(
            &positionSelectionHudLabelGraphics[positionSelectionHudIndicatorIndex]);
        delay(&positionSelectionHudLabelGraphics[positionSelectionHudIndicatorIndex]);
    }

    drivers->refSerial.sendGraphic(&positionSelectionHudLabelGraphics[NUM_POSITION_HUD_INDICATORS]);
    delay(&positionSelectionHudLabelGraphics[NUM_POSITION_HUD_INDICATORS]);

    for (reticleIndex = 0; reticleIndex < MODM_ARRAY_SIZE(reticleMsg); reticleIndex++)
    {
        drivers->refSerial.sendGraphic(&reticleMsg[reticleIndex]);
        delay(&reticleMsg[reticleIndex]);
    }

    drivers->refSerial.sendGraphic(&driveCommandMsg);
    delay(&driveCommandMsg);

    drivers->refSerial.sendGraphic(&chassisOrientationGraphics);
    delay(&chassisOrientationGraphics);
    chassisOrientationGraphics.graphicData[0].operation = Tx::ADD_GRAPHIC_MODIFY;
    chassisOrientationGraphics.graphicData[1].operation = Tx::ADD_GRAPHIC_MODIFY;

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

    booleanHudIndicatorDrawers[CV_AIM_DATA_VALID].setIndicatorState(
        drivers->legacyVisionCoprocessor.lastAimDataValid());

    if (agitatorSubsystem != nullptr)
    {
        booleanHudIndicatorDrawers[AGITATOR_STATUS_HEALTHY].setIndicatorState(
            !agitatorSubsystem->isJammed());
    }

    // TODO add hero fire ready and systems calibrating when available
    booleanHudIndicatorDrawers[SYSTEMS_CALIBRATING].setIndicatorState(true);

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

    for (positionSelectionHudIndicatorIndex = 0;
         positionSelectionHudIndicatorIndex < NUM_POSITION_HUD_INDICATORS;
         positionSelectionHudIndicatorIndex++)
    {
        RF_CALL(positionSelectionHudIndicatorDrawers[positionSelectionHudIndicatorIndex].draw());
    }

    RF_END();
}

int hopperIndex = 0;

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
    positionSelectionHudIndicatorDrawers[CHASSIS_STATE].setIndicatorState(
        POSITION_HUD_INDICATOR_LABELS_START_Y -
        1.5f * currDriveCommandIndex * POSITION_HUD_INDICATOR_LABEL_CHAR_SIZE -
        POSITION_HUD_INDICATOR_LABEL_CHAR_SIZE - POSITION_HUD_INDICATOR_SELECTOR_BOX_WIDTH - 1);

    // update flywheel and hopper state
    bool flywheelsOff =
        frictionWheelSubsystem != nullptr &&
        compareFloatClose(0.0f, frictionWheelSubsystem->getDesiredLaunchSpeed(), 1E-5);
    bool hopperOpen = hopperSubsystem != nullptr && hopperSubsystem->getIsHopperOpen();

    hopperIndex = 0;
    if (!flywheelsOff && hopperOpen)
    {
        hopperIndex = 1;
    }
    else if (flywheelsOff)
    {
        hopperIndex = 2;
    }
    positionSelectionHudIndicatorDrawers[FLYWHEEL_AND_HOPPER_STATE].setIndicatorState(
        POSITION_HUD_INDICATOR_LABELS_START_Y -
        1.5f * hopperIndex * POSITION_HUD_INDICATOR_LABEL_CHAR_SIZE -
        POSITION_HUD_INDICATOR_LABEL_CHAR_SIZE - POSITION_HUD_INDICATOR_SELECTOR_BOX_WIDTH - 1);
}

modm::ResumableResult<bool> ClientDisplayCommand::updateDriveCommandMsg()
{
    RF_BEGIN(3);

    prevDriveCommandIndex = currDriveCommandIndex;

    // Check if updating is necessary
    for (size_t i = 0; i < driveCommands.size(); i++)
    {
        if (drivers->commandScheduler.isCommandScheduled(driveCommands[i]))
        {
            currDriveCommandIndex = i;
        }
    }

    if (currDriveCommandIndex != prevDriveCommandIndex && currDriveCommandIndex != -1)
    {
        // Modify the graphic
        drivers->refSerial.configGraphicGenerics(
            &driveCommandMsg.graphicData,
            DRIVE_COMMAND_NAME,
            Tx::ADD_GRAPHIC_MODIFY,
            DRIVE_COMMAND_GRAPHIC_LAYER,
            static_cast<Tx::GraphicColor>(currDriveCommandIndex));

        drivers->refSerial.configCharacterMsg(
            DRIVE_COMMAND_CHAR_SIZE,
            DRIVE_COMMAND_CHAR_LINE_WIDTH,
            DRIVE_COMMAND_START_X,
            DRIVE_COMMAND_START_Y,
            driveCommands[currDriveCommandIndex]->getName(),
            &driveCommandMsg);

        drivers->refSerial.sendGraphic(&driveCommandMsg);

        delay(&driveCommandMsg);
    }

    // No delay necessary since didn't send anything
    RF_END();
}

modm::ResumableResult<bool> ClientDisplayCommand::updateChassisOrientation()
{
    RF_BEGIN(4);

    chassisOrientationRotated.rotate(modm::toRadian(
        turretSubsystem != nullptr ? -turretSubsystem->getYawAngleFromCenter() : 0.0f));

    if (chassisOrientationRotated != chassisOrientationPrev)
    {
        RefSerial::configLine(
            CHASSIS_LINE_WIDTH,
            CHASSIS_CENTER_X + chassisOrientationRotated.x,
            CHASSIS_CENTER_Y + chassisOrientationRotated.y,
            CHASSIS_CENTER_X - chassisOrientationRotated.x,
            CHASSIS_CENTER_Y - chassisOrientationRotated.y,
            &chassisOrientationGraphics.graphicData[0]);
        drivers->refSerial.sendGraphic(&chassisOrientationGraphics);
        delay(&chassisOrientationGraphics);

        chassisOrientationPrev = chassisOrientationRotated;
    }

    chassisOrientationRotated = chassisOrientation;

    RF_END();
}

modm::ResumableResult<bool> ClientDisplayCommand::updateTurretAngles()
{
    RF_BEGIN(5);

    if (turretSubsystem != nullptr)
    {
        yaw = drivers->turretMCBCanComm.getYaw();
#if defined(TARGET_HERO)
        pitch = turretSubsystem->getCurrentPitchValue().getValue();
#else
        pitch = drivers->turretMCBCanComm.getPitch();
#endif

        if (sendTurretDataTimer.execute() &&
            (!compareFloatClose(prevYaw, yaw, 1.0f / TURRET_ANGLES_DECIMAL_PRECISION) ||
             !compareFloatClose(prevPitch, pitch, 1.0f / TURRET_ANGLES_DECIMAL_PRECISION)))
        {
            bytesWritten = sprintf(
                turretAnglesGraphics.msg,
                "%i.%i\n\n%i.%i",
                static_cast<int>(yaw),
                abs(static_cast<int>(yaw * TURRET_ANGLES_DECIMAL_PRECISION) %
                    TURRET_ANGLES_DECIMAL_PRECISION),
                static_cast<int>(pitch),
                abs(static_cast<int>(pitch * TURRET_ANGLES_DECIMAL_PRECISION) %
                    TURRET_ANGLES_DECIMAL_PRECISION));
            turretAnglesGraphics.graphicData.endAngle = bytesWritten;

            drivers->refSerial.sendGraphic(&turretAnglesGraphics);
            delay(&turretAnglesGraphics);

            prevYaw = yaw;
            prevPitch = pitch;
        }
    }

    RF_END();
}

void ClientDisplayCommand::initializeBooleanHudIndicators()
{
    uint8_t booleanHudIndicatorName[3];
    memcpy(
        booleanHudIndicatorName,
        BOOLEAN_HUD_INDICATOR_LIST_START_NAME,
        sizeof(booleanHudIndicatorName));
    uint16_t hudIndicatorListCurrY = BOOLEAN_HUD_INDICATOR_LIST_START_Y;

    // Configure hopper cover hud indicator
    for (int i = 0; i < NUM_BOOLEAN_HUD_INDICATORS; i++)
    {
        RefSerial::configGraphicGenerics(
            &booleanHudIndicatorGraphics[i].graphicData,
            booleanHudIndicatorName,
            Tx::ADD_GRAPHIC,
            BOOLEAN_HUD_INDICATOR_LIST_LAYER,
            BOOLEAN_HUD_INDICATOR_FILLED_COLOR);

        RefSerial::configCircle(
            BOOLEAN_HUD_INDICATOR_WIDTH,
            BOOLEAN_HUD_INDICATOR_LIST_CENTER_X,
            hudIndicatorListCurrY,
            BOOLEAN_HUD_INDICATOR_RADIUS,
            &booleanHudIndicatorGraphics[i].graphicData);

        booleanHudIndicatorName[2]++;

        RefSerial::configGraphicGenerics(
            &booleanHudIndicatorStaticGrahpics[i].graphicData,
            booleanHudIndicatorName,
            Tx::ADD_GRAPHIC,
            BOOLEAN_HUD_INDICATOR_STATIC_LIST_LAYER,
            BOOLEAN_HUD_INDICATOR_OUTLINE_COLOR);

        RefSerial::configCircle(
            BOOLEAN_HUD_INDICATOR_OUTLINE_WIDTH,
            BOOLEAN_HUD_INDICATOR_LIST_CENTER_X,
            hudIndicatorListCurrY,
            BOOLEAN_HUD_INDICATOR_OUTLINE_RADIUS,
            &booleanHudIndicatorStaticGrahpics[i].graphicData);

        booleanHudIndicatorName[2]++;

        RefSerial::configGraphicGenerics(
            &booleanHudIndicatorStaticLabelGraphics[i].graphicData,
            booleanHudIndicatorName,
            Tx::ADD_GRAPHIC,
            BOOLEAN_HUD_INDICATOR_STATIC_LIST_LAYER,
            BOOLEAN_HUD_INDICATOR_LABEL_COLOR);

        RefSerial::configCharacterMsg(
            BOOLEAN_HUD_INDICATOR_LABEL_CHAR_SIZE,
            BOOLEAN_HUD_INDICATOR_LABEL_CHAR_LINE_WIDTH,
            BOOLEAN_HUD_INDICATOR_LIST_CENTER_X -
                strlen(BOOLEAN_HUD_INDICATOR_LABELS[i]) * BOOLEAN_HUD_INDICATOR_LABEL_CHAR_SIZE -
                BOOLEAN_HUD_INDICATOR_OUTLINE_RADIUS - BOOLEAN_HUD_INDICATOR_OUTLINE_WIDTH / 2,
            hudIndicatorListCurrY + BOOLEAN_HUD_INDICATOR_LABEL_CHAR_SIZE / 2,
            BOOLEAN_HUD_INDICATOR_LABELS[i],
            &booleanHudIndicatorStaticLabelGraphics[i]);

        booleanHudIndicatorName[2]++;

        hudIndicatorListCurrY -= BOOLEAN_HUD_INDICATOR_LIST_DIST_BTWN_BULLETS;
    }
}
void ClientDisplayCommand::initializePositionHudIndicators()
{
    uint8_t positionHudIndicatorName[3];
    memcpy(
        positionHudIndicatorName,
        POSITION_HUD_INDICATOR_START_NAME,
        sizeof(positionHudIndicatorName));

    uint16_t hudIndicatorListCurrX = POSITION_HUD_INDICATOR_START_X;

    // Configure hopper cover hud indicator
    for (int i = 0; i < NUM_POSITION_HUD_INDICATORS; i++)
    {
        // configure rectangle

        RefSerial::configGraphicGenerics(
            &positionSelectionHudIndicatorGraphics[i].graphicData,
            positionHudIndicatorName,
            Tx::ADD_GRAPHIC,
            POSITION_HUD_INDICATOR_LAYER,
            POSITION_HUD_INDICATOR_SELECTOR_BOX_COLOR);

        positionHudIndicatorName[2]++;

        RefSerial::configRectangle(
            POSITION_HUD_INDICATOR_SELECTOR_BOX_WIDTH,
            hudIndicatorListCurrX - 2 * POSITION_HUD_INDICATOR_SELECTOR_BOX_WIDTH - 1,
            POSITION_HUD_INDICATOR_LABELS_START_Y - POSITION_HUD_INDICATOR_LABEL_CHAR_SIZE -
                POSITION_HUD_INDICATOR_SELECTOR_BOX_WIDTH - 1,
            hudIndicatorListCurrX +
                POSITION_HUD_INDICATOR_TITLE_WIDTH * POSITION_HUD_INDICATOR_LABEL_CHAR_SIZE +
                POSITION_HUD_INDICATOR_SELECTOR_BOX_WIDTH + 1,
            POSITION_HUD_INDICATOR_LABELS_START_Y + POSITION_HUD_INDICATOR_SELECTOR_BOX_WIDTH + 1,
            &positionSelectionHudIndicatorGraphics[i].graphicData);

        // configure labels

        RefSerial::configGraphicGenerics(
            &positionSelectionHudLabelGraphics[i].graphicData,
            positionHudIndicatorName,
            Tx::ADD_GRAPHIC,
            POSITION_HUD_INDICATOR_LAYER,
            POSITION_HUD_INDICATOR_LABELS_COLOR);

        positionHudIndicatorName[2]++;

        RefSerial::configCharacterMsg(
            POSITION_HUD_INDICATOR_LABEL_CHAR_SIZE,
            POSITION_HUD_INDICATOR_LABEL_CHAR_LINE_WIDTH,
            hudIndicatorListCurrX,
            POSITION_HUD_INDICATOR_LABELS_START_Y,
            POSITION_HUD_INDICATOR_LABELS[i],
            &positionSelectionHudLabelGraphics[i]);

        hudIndicatorListCurrX +=
            POSITION_HUD_INDICATOR_TITLE_WIDTH * POSITION_HUD_INDICATOR_LABEL_CHAR_SIZE +
            POSITION_HUD_INDICATOR_DIST_BTWN_INDICATOR_COLS;
    }

    // title

    RefSerial::configGraphicGenerics(
        &positionSelectionHudLabelGraphics[NUM_POSITION_HUD_INDICATORS].graphicData,
        positionHudIndicatorName,
        Tx::ADD_GRAPHIC,
        POSITION_HUD_INDICATOR_LAYER,
        POSITION_HUD_INDICATOR_TITLE_COLOR);

    char positionHudGraphicTitles[30];
    char *currHudGraphicTitlePos = positionHudGraphicTitles;

    const int spacesBetweenCols =
        POSITION_HUD_INDICATOR_DIST_BTWN_INDICATOR_COLS / POSITION_HUD_INDICATOR_LABEL_CHAR_SIZE;

    for (int i = 0; i < NUM_POSITION_HUD_INDICATORS; i++)
    {
        strcpy(currHudGraphicTitlePos, POSITION_HUD_INDICATOR_TITLES[i]);
        currHudGraphicTitlePos += POSITION_HUD_INDICATOR_TITLE_WIDTH;

        if (i != NUM_POSITION_HUD_INDICATORS - 1)
        {
            for (int j = 0; j < spacesBetweenCols; j++)
            {
                *currHudGraphicTitlePos = ' ';
                currHudGraphicTitlePos++;
            }
        }
    }

    RefSerial::configCharacterMsg(
        POSITION_HUD_INDICATOR_LABEL_CHAR_SIZE,
        POSITION_HUD_INDICATOR_LABEL_CHAR_LINE_WIDTH,
        POSITION_HUD_INDICATOR_START_X,
        POSITION_HUD_INDICATOR_TITLE_START_Y,
        positionHudGraphicTitles,
        &positionSelectionHudLabelGraphics[NUM_POSITION_HUD_INDICATORS]);
}

void ClientDisplayCommand::initializeReticle()
{
    uint8_t currLineName[3];
    memcpy(currLineName, RETICLE_LINES_START_NAME, sizeof(currLineName));

    // Add reticle markers
    uint16_t maxReticleY = 0;
    uint16_t minReticleY = UINT16_MAX;

    for (size_t i = 0; i < NUM_RETICLE_COORDINATES; i++)
    {
        size_t reticleMsgIndex = i / MODM_ARRAY_SIZE(reticleMsg[0].graphicData);
        size_t graphicDataIndex = i % MODM_ARRAY_SIZE(reticleMsg[0].graphicData);

        RefSerial::configGraphicGenerics(
            &reticleMsg[reticleMsgIndex].graphicData[graphicDataIndex],
            currLineName,
            Tx::ADD_GRAPHIC,
            RETICLE_GRAPHIC_LAYER,
            std::get<2>(TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES[i]));

        currLineName[2]++;

        uint16_t reticleXCenter = static_cast<int>(SCREEN_WIDTH / 2) + RETICLE_CENTER_X_OFFSET;

        uint16_t startX =
            reticleXCenter - std::get<0>(TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES[i]);
        uint16_t endX =
            reticleXCenter + std::get<0>(TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES[i]);

        uint16_t y = std::get<1>(TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES[i]);

        RefSerial::configLine(
            RETICLE_THICKNESS,
            startX,
            y,
            endX,
            y,
            &reticleMsg[reticleMsgIndex].graphicData[graphicDataIndex]);

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
        RETICLE_GRAPHIC_LAYER,
        RETICLE_HORIZONTAL_COLOR);

    RefSerial::configLine(
        RETICLE_THICKNESS,
        SCREEN_WIDTH / 2 + RETICLE_CENTER_X_OFFSET,
        minReticleY,
        SCREEN_WIDTH / 2 + RETICLE_CENTER_X_OFFSET,
        maxReticleY,
        &reticleMsg[NUM_RETICLE_COORDINATES / MODM_ARRAY_SIZE(reticleMsg[0].graphicData)]
             .graphicData[NUM_RETICLE_COORDINATES % MODM_ARRAY_SIZE(reticleMsg[0].graphicData)]);
}

void ClientDisplayCommand::initializeDriveCommand()
{
    drivers->refSerial.configGraphicGenerics(
        &driveCommandMsg.graphicData,
        DRIVE_COMMAND_NAME,
        Tx::ADD_GRAPHIC,
        DRIVE_COMMAND_GRAPHIC_LAYER,
        Tx::GraphicColor::WHITE);

    drivers->refSerial.configCharacterMsg(
        DRIVE_COMMAND_CHAR_SIZE,
        DRIVE_COMMAND_CHAR_LINE_WIDTH,
        DRIVE_COMMAND_START_X,
        DRIVE_COMMAND_START_Y,
        "",
        &driveCommandMsg);

    currDriveCommandIndex = -1;
    prevDriveCommandIndex = -1;
}

void ClientDisplayCommand::initializeChassisOrientation()
{
    chassisOrientation.set(0, CHASSIS_HEIGHT / 2);
    chassisOrientationPrev = chassisOrientation;
    chassisOrientationRotated = chassisOrientation;

    uint8_t chassisOrientationName[3];
    memcpy(chassisOrientationName, CHASSIS_ORIENTATION_START_NAME, sizeof(chassisOrientationName));

    RefSerial::configGraphicGenerics(
        &chassisOrientationGraphics.graphicData[0],
        chassisOrientationName,
        Tx::ADD_GRAPHIC,
        CHASSIS_ORIENTATION_LAYER,
        CHASSIS_ORIENTATION_COLOR);

    RefSerial::configLine(
        CHASSIS_LINE_WIDTH,
        CHASSIS_CENTER_X + chassisOrientation.x,
        CHASSIS_CENTER_Y + chassisOrientation.y,
        CHASSIS_CENTER_X - chassisOrientation.x,
        CHASSIS_CENTER_Y - chassisOrientation.y,
        &chassisOrientationGraphics.graphicData[0]);

    chassisOrientationName[2]++;

    RefSerial::configGraphicGenerics(
        &chassisOrientationGraphics.graphicData[1],
        chassisOrientationName,
        Tx::ADD_GRAPHIC,
        CHASSIS_ORIENTATION_LAYER,
        CHASSIS_BARREL_COLOR);

    RefSerial::configLine(
        CHASSIS_BARREL_LINE_WIDTH,
        CHASSIS_CENTER_X,
        CHASSIS_CENTER_Y,
        CHASSIS_CENTER_X,
        CHASSIS_CENTER_Y + CHASSIS_BARREL_LENGTH,
        &chassisOrientationGraphics.graphicData[1]);
}

void ClientDisplayCommand::initializeTurretAngles()
{
    if (turretSubsystem != nullptr)
    {
        uint8_t turretAnglesName[3];
        memcpy(turretAnglesName, TURRET_ANGLES_START_NAME, sizeof(turretAnglesName));

        RefSerial::configGraphicGenerics(
            &turretAnglesGraphics.graphicData,
            turretAnglesName,
            Tx::ADD_GRAPHIC,
            TURRET_ANGLES_LAYER,
            TURRET_ANGLES_COLOR);

        RefSerial::configCharacterMsg(
            TURRET_ANGLES_FONT_SIZE,
            TURRET_ANGLES_WIDTH,
            TURRET_ANGLES_START_X,
            TURRET_ANGLES_START_Y,
            "0\n\n0",
            &turretAnglesGraphics);

        turretAnglesName[2]++;

        RefSerial::configGraphicGenerics(
            &turretAnglesLabelGraphics.graphicData,
            turretAnglesName,
            Tx::ADD_GRAPHIC,
            TURRET_ANGLES_LAYER,
            TURRET_ANGLES_COLOR);

        RefSerial::configCharacterMsg(
            TURRET_ANGLES_FONT_SIZE,
            TURRET_ANGLES_WIDTH,
            TURRET_ANGLES_START_X - strlen("PITCH: ") * TURRET_ANGLES_FONT_SIZE,
            TURRET_ANGLES_START_Y,
            "  YAW:\n\nPITCH:",
            &turretAnglesLabelGraphics);

        prevYaw = 0.0f;
        prevPitch = 0.0f;
    }
}

}  // namespace aruwsrc::display
