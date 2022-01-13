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
using namespace tap::communication::serial;
using namespace aruwsrc::control;

#define delay()                                  \
    delayTimer.restart(DELAY_PERIOD_BTWN_SENDS); \
    RF_WAIT_UNTIL(delayTimer.execute());

namespace aruwsrc::display
{
ClientDisplayCommand::ClientDisplayCommand(
    aruwsrc::Drivers *drivers,
    ClientDisplaySubsystem *clientDisplay,
    const TurretMCBHopperSubsystem *hopperSubsystem,
    const aruwsrc::control::launcher::FrictionWheelSubsystem *frictionWheelSubsystem,
    const Command *wiggleCommand,
    const Command *followTurretCommand,
    const Command *beybladeCommand,
    const Command *baseDriveCommand)
    : Command(),
      drivers(drivers),
      hopperSubsystem(hopperSubsystem),
      frictionWheelSubsystem(frictionWheelSubsystem),
      bubbleDrawers({
          BubbleDrawer(drivers, &bubbleGraphics[0]),
          BubbleDrawer(drivers, &bubbleGraphics[1]),
          BubbleDrawer(drivers, &bubbleGraphics[2]),
      }),
      wiggleCommand(wiggleCommand),
      followTurretCommand(followTurretCommand),
      beybladeCommand(beybladeCommand),
      baseDriveCommand(baseDriveCommand),
      driveCommandMsg()
{
    addSubsystemRequirement(clientDisplay);
}

void ClientDisplayCommand::initialize()
{
    restart();  // restart protothread
    initializeBubbles();
    initializeReticle();
    initializeDriveCommand();
}

void ClientDisplayCommand::execute() { run(); }

bool ClientDisplayCommand::run()
{
    PT_BEGIN();

    PT_CALL(initializeNonblocking());

    while (true)
    {
        bubbleDrawers[HOPPER_OPEN].setBubbleFilled(!hopperSubsystem->getIsHopperOpen());
        bubbleDrawers[FRICTION_WHEELS_ON].setBubbleFilled(
            !compareFloatClose(0.0f, frictionWheelSubsystem->getDesiredLaunchSpeed(), 1E-5));
        bubbleDrawers[CV_ENABLED].setBubbleFilled(
            drivers->legacyVisionCoprocessor.lastAimDataValid());

        for (bubbleIndex = 0; bubbleIndex < NUM_BUBBLES; bubbleIndex++)
        {
            PT_CALL(bubbleDrawers[bubbleIndex].draw());
        }

        PT_CALL(updateDriveCommandMsg());

        PT_YIELD();
    }
    PT_END();
}

modm::ResumableResult<bool> ClientDisplayCommand::initializeNonblocking()
{
    RF_BEGIN(0);

    RF_WAIT_WHILE(drivers->refSerial.getRobotData().robotId == RefSerial::RobotId::INVALID);

    for (bubbleIndex = 0; bubbleIndex < NUM_BUBBLES; bubbleIndex++)
    {
        RF_CALL(bubbleDrawers[bubbleIndex].initialize());
        drivers->refSerial.sendGraphic(&bubbleStaticGraphics[bubbleIndex]);
        delay();
        drivers->refSerial.sendGraphic(&bubbleStaticLabelGraphics[bubbleIndex]);
        delay();
    }

    delay();
    drivers->refSerial.sendGraphic(&reticleMsg);
    delay();
    drivers->refSerial.sendGraphic(&driveCommandMsg);
    delay();

    RF_END();
}

modm::ResumableResult<bool> ClientDisplayCommand::updateDriveCommandMsg()
{
    RF_BEGIN(1);
    // Check if updating is necessary
    if (drivers->commandScheduler.isCommandScheduled(wiggleCommand))
    {
        newDriveCommandScheduled = wiggleCommand;
        driveCommandColor = RefSerial::Tx::GraphicColor::YELLOW;
    }
    else if (drivers->commandScheduler.isCommandScheduled(followTurretCommand))
    {
        newDriveCommandScheduled = followTurretCommand;
        driveCommandColor = RefSerial::Tx::GraphicColor::ORANGE;
    }
    else if (drivers->commandScheduler.isCommandScheduled(beybladeCommand))
    {
        newDriveCommandScheduled = beybladeCommand;
        driveCommandColor = RefSerial::Tx::GraphicColor::PURPLISH_RED;
    }
    else if (drivers->commandScheduler.isCommandScheduled(baseDriveCommand))
    {
        newDriveCommandScheduled = baseDriveCommand;
        driveCommandColor = RefSerial::Tx::GraphicColor::GREEN;
    }

    if ((newDriveCommandScheduled != currDriveCommandScheduled &&
         newDriveCommandScheduled != nullptr))
    {
        currDriveCommandScheduled = newDriveCommandScheduled;

        // Modify the graphic
        drivers->refSerial.configGraphicGenerics(
            &driveCommandMsg.graphicData,
            DRIVE_TEXT_NAME,
            RefSerial::Tx::AddGraphicOperation::ADD_GRAPHIC_MODIFY,
            DRIVE_COMMAND_GRAPHIC_LAYER,
            driveCommandColor);

        drivers->refSerial.configCharacterMsg(
            FONT_SIZE,
            400,
            LINE_THICKNESS,
            SCREEN_MARGIN,
            TEXT_TOP_ROW_Y,
            currDriveCommandScheduled->getName(),
            &driveCommandMsg);

        drivers->refSerial.sendGraphic(&driveCommandMsg);
        delay();
    }

    // No delay necessary since didn't send anything
    RF_END();
}

modm::ResumableResult<bool> ClientDisplayCommand::updateCapBankMsg()
{
    RF_BEGIN(2);

    if (sendCapBankTimer.execute())
    {
        drivers->refSerial.sendGraphic(&capStringMsg, false, true);

        delay();

        capPowerRemainMsg.graphicData.operation = RefSerial::Tx::AddGraphicOperation::ADD_GRAPHIC;
        drivers->refSerial.updateInteger(capicatance++, &capPowerRemainMsg.graphicData);
        drivers->refSerial.sendGraphic(&capPowerRemainMsg);
        capPowerRemainMsg.graphicData.operation =
            RefSerial::Tx::AddGraphicOperation::ADD_GRAPHIC_MODIFY;
    }
    else
    {
        drivers->refSerial.updateInteger(capicatance++, &capPowerRemainMsg.graphicData);
        drivers->refSerial.sendGraphic(&capPowerRemainMsg);
    }

    delay();
    RF_END();
}

modm::ResumableResult<bool> ClientDisplayCommand::updateTurretReticleMsg()
{
    RF_BEGIN(3);
    delay();
    if (sendReticleTimer.execute())
    {
        drivers->refSerial.sendGraphic(&reticleMsg, false, true);
        delay();
    }
    RF_END();
}

void ClientDisplayCommand::initCapBankMsg()
{
    drivers->refSerial.configGraphicGenerics(
        &capStringMsg.graphicData,
        CAP_TEXT_NAME,
        RefSerial::Tx::AddGraphicOperation::ADD_GRAPHIC,
        CAP_BANK_LAYER_1,
        RefSerial::Tx::GraphicColor::YELLOW);

    drivers->refSerial.configCharacterMsg(
        FONT_SIZE,
        200,
        FONT_THICKNESS,
        SCREEN_WIDTH - 400,
        TEXT_TOP_ROW_Y,
        "Capacitance:",
        &capStringMsg);
    drivers->refSerial.sendGraphic(&capStringMsg, true, false);

    drivers->refSerial.configGraphicGenerics(
        &capPowerRemainMsg.graphicData,
        CAP_VALUE_NAME,
        RefSerial::Tx::AddGraphicOperation::ADD_GRAPHIC_MODIFY,
        CAP_BANK_LAYER_2,
        RefSerial::Tx::GraphicColor::YELLOW);

    drivers->refSerial.configInteger(
        FONT_SIZE + 10,  // Slightly larger than other text
        FONT_THICKNESS,
        SCREEN_WIDTH - 350,
        TEXT_TOP_ROW_Y - 60,
        capicatance++,
        &capPowerRemainMsg.graphicData);
}

void ClientDisplayCommand::initDriveCommandMsg() {}

void ClientDisplayCommand::initTurretReticleMsg() { drivers->refSerial.sendGraphic(&reticleMsg); }

void ClientDisplayCommand::initializeBubbles()
{
    uint8_t bubbleName[3];
    memcpy(bubbleName, BUBBLE_LIST_START_NAME, sizeof(bubbleName));
    uint16_t bubbleListCurrY = BUBBLE_LIST_START_Y;

    // Configure hopper cover bubble
    for (bubbleIndex = 0; bubbleIndex < NUM_BUBBLES; bubbleIndex++)
    {
        RefSerial::configGraphicGenerics(
            &bubbleGraphics[bubbleIndex].graphicData,
            bubbleName,
            RefSerial::Tx::ADD_GRAPHIC,
            BUBBLE_LIST_LAYER,
            BUBBLE_FILLED_COLOR);

        RefSerial::configCircle(
            BUBBLE_WIDTH,
            BUBBLE_LIST_CENTER_X,
            bubbleListCurrY,
            BUBBLE_RADIUS,
            &bubbleGraphics[bubbleIndex].graphicData);

        bubbleName[2]++;

        RefSerial::configGraphicGenerics(
            &bubbleStaticGraphics[bubbleIndex].graphicData,
            bubbleName,
            RefSerial::Tx::ADD_GRAPHIC,
            BUBBLE_STATIC_LIST_LAYER,
            BUBBLE_OUTLINE_COLOR);

        RefSerial::configCircle(
            BUBBLE_OUTLINE_WIDTH,
            BUBBLE_LIST_CENTER_X,
            bubbleListCurrY,
            BUBBLE_OUTLINE_RADIUS,
            &bubbleStaticGraphics[bubbleIndex].graphicData);

        bubbleName[2]++;

        RefSerial::configGraphicGenerics(
            &bubbleStaticLabelGraphics[bubbleIndex].graphicData,
            bubbleName,
            RefSerial::Tx::ADD_GRAPHIC,
            BUBBLE_STATIC_LIST_LAYER,
            BUBBLE_LABEL_COLOR);

        RefSerial::configCharacterMsg(
            BUBBLE_LABEL_CHAR_WIDTH,
            BUBBLE_LABEL_CHAR_LENGTH,
            BUBBLE_LABEL_CHAR_LINE_WIDTH,
            BUBBLE_LIST_CENTER_X - strlen(BUBBLE_LABELS[bubbleIndex]) * BUBBLE_LABEL_CHAR_WIDTH -
                BUBBLE_OUTLINE_RADIUS - BUBBLE_OUTLINE_WIDTH / 2,
            bubbleListCurrY + BUBBLE_LABEL_CHAR_WIDTH / 2,
            BUBBLE_LABELS[bubbleIndex],
            &bubbleStaticLabelGraphics[bubbleIndex]);

        bubbleName[2]++;

        bubbleListCurrY += BUBBLE_LIST_DIST_BTWN_BULLETS;
    }
}

void ClientDisplayCommand::initializeReticle()
{
    drivers->refSerial.configGraphicGenerics(
        &reticleMsg.graphicData[0],
        RETICLE_LINE1_NAME,
        RefSerial::Tx::AddGraphicOperation::ADD_GRAPHIC,
        RETICLE_GRAPHIC_LAYER,
        RefSerial::Tx::GraphicColor::YELLOW);

    drivers->refSerial.configGraphicGenerics(
        &reticleMsg.graphicData[1],
        RETICLE_LINE2_NAME,
        RefSerial::Tx::AddGraphicOperation::ADD_GRAPHIC,
        RETICLE_GRAPHIC_LAYER,
        RefSerial::Tx::GraphicColor::YELLOW);

    drivers->refSerial.configGraphicGenerics(
        &reticleMsg.graphicData[2],
        RETICLE_LINE3_NAME,
        RefSerial::Tx::AddGraphicOperation::ADD_GRAPHIC,
        RETICLE_GRAPHIC_LAYER,
        RefSerial::Tx::GraphicColor::YELLOW);

    drivers->refSerial.configGraphicGenerics(
        &reticleMsg.graphicData[3],
        RETICLE_LINE4_NAME,
        RefSerial::Tx::AddGraphicOperation::ADD_GRAPHIC,
        RETICLE_GRAPHIC_LAYER,
        RefSerial::Tx::GraphicColor::YELLOW);

    drivers->refSerial.configGraphicGenerics(
        &reticleMsg.graphicData[4],
        RETICLE_CIRCLE_NAME,
        RefSerial::Tx::AddGraphicOperation::ADD_GRAPHIC,
        RETICLE_GRAPHIC_LAYER,
        RefSerial::Tx::GraphicColor::YELLOW);

    drivers->refSerial.configLine(
        RETICLE_THICKNESS,
        SCREEN_WIDTH / 2 - TURRET_RETICLE_1M_WIDTH / 2,
        TURRET_RETICLE_1MY,
        SCREEN_WIDTH / 2 + TURRET_RETICLE_1M_WIDTH / 2,
        TURRET_RETICLE_1MY,
        &reticleMsg.graphicData[0]);

    drivers->refSerial.configLine(
        RETICLE_THICKNESS,
        SCREEN_WIDTH / 2 - TURRET_RETICLE_3M_WIDTH / 2,
        TURRET_RETICLE_3MY,
        SCREEN_WIDTH / 2 + TURRET_RETICLE_3M_WIDTH / 2,
        TURRET_RETICLE_3MY,
        &reticleMsg.graphicData[1]);

    drivers->refSerial.configLine(
        RETICLE_THICKNESS,
        SCREEN_WIDTH / 2 - TURRET_RETICLE_5M_WIDTH / 2,
        TURRET_RETICLE_5MY,
        SCREEN_WIDTH / 2 + TURRET_RETICLE_5M_WIDTH / 2,
        TURRET_RETICLE_5MY,
        &reticleMsg.graphicData[2]);

    drivers->refSerial.configLine(
        RETICLE_THICKNESS,
        SCREEN_WIDTH / 2,
        TURRET_RETICLE_1MY,
        SCREEN_WIDTH / 2,
        TURRET_RETICLE_5MY - 50,
        &reticleMsg.graphicData[3]);

    drivers->refSerial.configCircle(
        RETICLE_THICKNESS,
        SCREEN_WIDTH / 2,
        TURRET_RETICLE_1MY,
        5,
        &reticleMsg.graphicData[4]);
}

void ClientDisplayCommand::initializeDriveCommand()
{
    drivers->refSerial.configGraphicGenerics(
        &driveCommandMsg.graphicData,
        DRIVE_TEXT_NAME,
        RefSerial::Tx::AddGraphicOperation::ADD_GRAPHIC,
        DRIVE_COMMAND_GRAPHIC_LAYER,
        driveCommandColor);

    drivers->refSerial.configCharacterMsg(
        FONT_SIZE,
        400,
        LINE_THICKNESS,
        SCREEN_MARGIN,
        TEXT_TOP_ROW_Y,
        "",
        &driveCommandMsg);
}

}  // namespace aruwsrc::display
