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

#define delay()                                  \
    delayTimer.restart(DELAY_PERIOD_BTWN_SENDS); \
    RF_WAIT_UNTIL(delayTimer.execute());

namespace aruwsrc::display
{
ClientDisplayCommand::ClientDisplayCommand(
    aruwsrc::Drivers *drivers,
    ClientDisplaySubsystem *clientDisplay,
    const TurretMCBHopperSubsystem *hopperSubsystem,
    const FrictionWheelSubsystem *frictionWheelSubsystem,
    AgitatorSubsystem *agitatorSubsystem,
    const Command *wiggleCommand,
    const Command *followTurretCommand,
    const Command *beybladeCommand,
    const Command *baseDriveCommand)
    : Command(),
      drivers(drivers),
      hopperSubsystem(hopperSubsystem),
      frictionWheelSubsystem(frictionWheelSubsystem),
      agitatorSubsystem(agitatorSubsystem),
      bubbleDrawers{
          BooleanDrawer(drivers, &bubbleGraphics[0]),
          BooleanDrawer(drivers, &bubbleGraphics[1]),
          BooleanDrawer(drivers, &bubbleGraphics[2]),
          BooleanDrawer(drivers, &bubbleGraphics[3]),
      },
#if defined(TARGET_HERO)
      bubbleDrawers{
          BooleanDrawer(drivers, &bubbleGraphics[0]),
          BooleanDrawer(drivers, &bubbleGraphics[1]),
          BooleanDrawer(drivers, &bubbleGraphics[2]),
          BooleanDrawer(drivers, &bubbleGraphics[3]),
          BooleanDrawer(drivers, &bubbleGraphics[4]),
      },
#endif
      driveCommands{
          wiggleCommand,
          followTurretCommand,
          beybladeCommand,
          baseDriveCommand,
      }
{
    addSubsystemRequirement(clientDisplay);
    bubbleDrawers[AGITATOR_STATUS_HEALTHY].setBoolFalseColor(Tx::GraphicColor::PURPLISH_RED);
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
        bubbleDrawers[HOPPER_OPEN].setDrawerColor(!hopperSubsystem->getIsHopperOpen());
        bubbleDrawers[FRICTION_WHEELS_ON].setDrawerColor(
            !compareFloatClose(0.0f, frictionWheelSubsystem->getDesiredLaunchSpeed(), 1E-5));
        bubbleDrawers[CV_AIM_DATA_VALID].setDrawerColor(
            drivers->legacyVisionCoprocessor.lastAimDataValid());
        bubbleDrawers[AGITATOR_STATUS_HEALTHY].setDrawerColor(!agitatorSubsystem->isJammed());

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

    for (reticleIndex = 0; reticleIndex < MODM_ARRAY_SIZE(reticleMsg); reticleIndex++)
    {
        drivers->refSerial.sendGraphic(&reticleMsg[reticleIndex]);
        delay();
    }

    drivers->refSerial.sendGraphic(&driveCommandMsg);
    delay();

    RF_END();
}

modm::ResumableResult<bool> ClientDisplayCommand::updateDriveCommandMsg()
{
    RF_BEGIN(1);

    prevDriveCommandIndex = currDriveCommandIndex;

    // Check if updating is necessary
    for (size_t i = 0; i < MODM_ARRAY_SIZE(driveCommands); i++)
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
            Tx::AddGraphicOperation::ADD_GRAPHIC_MODIFY,
            DRIVE_COMMAND_GRAPHIC_LAYER,
            driveCommandColor);

        drivers->refSerial.configCharacterMsg(
            DRIVE_COMMAND_CHAR_SIZE,
            DRIVE_COMMAND_CHAR_LENGTH,
            DRIVE_COMMAND_CHAR_LINE_WIDTH,
            DRIVE_COMMAND_START_X,
            DRIVE_COMMAND_START_Y,
            driveCommands[currDriveCommandIndex]->getName(),
            &driveCommandMsg);

        drivers->refSerial.sendGraphic(&driveCommandMsg);

        delay();
    }

    // No delay necessary since didn't send anything
    RF_END();
}

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
            Tx::ADD_GRAPHIC,
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
            Tx::ADD_GRAPHIC,
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
            Tx::ADD_GRAPHIC,
            BUBBLE_STATIC_LIST_LAYER,
            BUBBLE_LABEL_COLOR);

        RefSerial::configCharacterMsg(
            BUBBLE_LABEL_CHAR_SIZE,
            BUBBLE_LABEL_CHAR_LENGTH,
            BUBBLE_LABEL_CHAR_LINE_WIDTH,
            BUBBLE_LIST_CENTER_X - strlen(BUBBLE_LABELS[bubbleIndex]) * BUBBLE_LABEL_CHAR_SIZE -
                BUBBLE_OUTLINE_RADIUS - BUBBLE_OUTLINE_WIDTH / 2,
            bubbleListCurrY + BUBBLE_LABEL_CHAR_SIZE / 2,
            BUBBLE_LABELS[bubbleIndex],
            &bubbleStaticLabelGraphics[bubbleIndex]);

        bubbleName[2]++;

        bubbleListCurrY += BUBBLE_LIST_DIST_BTWN_BULLETS;
    }
}

void ClientDisplayCommand::initializeReticle()
{
    uint8_t currLineName[3];
    memcpy(currLineName, RETICLE_LINES_START_NAME, sizeof(currLineName));

    for (size_t i = 0; i < NUM_RETICLE_COORDINATES; i++)
    {
        size_t reticleMsgIndex = i / MODM_ARRAY_SIZE(reticleMsg[0].graphicData);
        size_t graphicDataIndex = i % MODM_ARRAY_SIZE(reticleMsg[0].graphicData);

        RefSerial::configGraphicGenerics(
            &reticleMsg[reticleMsgIndex].graphicData[graphicDataIndex],
            currLineName,
            Tx::AddGraphicOperation::ADD_GRAPHIC,
            RETICLE_GRAPHIC_LAYER,
            std::get<2>(TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES[i]));

        currLineName[2]++;

        uint16_t reticleXCenter = SCREEN_WIDTH / 2 + RETICLE_CENTER_X_OFFSET;

        uint16_t startX =
            reticleXCenter - std::get<0>(TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES[i]);
        uint16_t endX =
            reticleXCenter - std::get<0>(TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES[i]);

        uint16_t y = std::get<1>(TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES[i]);

        drivers->refSerial.configLine(
            RETICLE_THICKNESS,
            startX,
            y,
            endX,
            y,
            &reticleMsg[reticleMsgIndex].graphicData[graphicDataIndex]);
    }
}

void ClientDisplayCommand::initializeDriveCommand()
{
    drivers->refSerial.configGraphicGenerics(
        &driveCommandMsg.graphicData,
        DRIVE_COMMAND_NAME,
        Tx::AddGraphicOperation::ADD_GRAPHIC,
        DRIVE_COMMAND_GRAPHIC_LAYER,
        Tx::GraphicColor::WHITE);

    drivers->refSerial.configCharacterMsg(
        DRIVE_COMMAND_CHAR_SIZE,
        DRIVE_COMMAND_CHAR_LENGTH,
        DRIVE_COMMAND_CHAR_LINE_WIDTH,
        DRIVE_COMMAND_START_X,
        DRIVE_COMMAND_START_Y,
        "",
        &driveCommandMsg);

    currDriveCommandIndex = -1;
    prevDriveCommandIndex = -1;
}

}  // namespace aruwsrc::display
