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
    const aruwsrc::control::turret::TurretSubsystem *turretSubsystem,
    const std::vector<const Command *> &driveCommands)
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
      driveCommands{driveCommands},
      turretSubsystem(turretSubsystem)
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
    initializeVehicleOrientation();
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

        PT_CALL(updateVehicleOrientation());

        PT_YIELD();
    }
    PT_END();
}

modm::ResumableResult<bool> ClientDisplayCommand::initializeNonblocking()
{
    RF_BEGIN(0);

    RF_WAIT_UNTIL(drivers->refSerial.getRefSerialReceivingData());

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

    drivers->refSerial.sendGraphic(&chassisOrientationGraphics);
    chassisOrientationGraphics.graphicData[0].operation =
        Tx::AddGraphicOperation::ADD_GRAPHIC_MODIFY;
    chassisOrientationGraphics.graphicData[1].operation =
        Tx::AddGraphicOperation::ADD_GRAPHIC_MODIFY;

    RF_END();
}

modm::ResumableResult<bool> ClientDisplayCommand::updateDriveCommandMsg()
{
    RF_BEGIN(1);

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
            Tx::AddGraphicOperation::ADD_GRAPHIC_MODIFY,
            DRIVE_COMMAND_GRAPHIC_LAYER,
            static_cast<Tx::GraphicColor>(currDriveCommandIndex));

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

modm::ResumableResult<bool> ClientDisplayCommand::updateVehicleOrientation()
{
    RF_BEGIN(2)

    chassisOrientationRotated.rotate(modm::toRadian(-turretSubsystem->getYawAngleFromCenter()));

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
        delay();

        chassisOrientationPrev = chassisOrientationRotated;
    }

    chassisOrientationRotated = chassisOrientation;

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
            Tx::AddGraphicOperation::ADD_GRAPHIC,
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
        Tx::AddGraphicOperation::ADD_GRAPHIC,
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

void ClientDisplayCommand::initializeVehicleOrientation()
{
    chassisOrientation.set(0, CHASSIS_HEIGHT / 2);
    chassisOrientationPrev = chassisOrientation;
    chassisOrientationRotated = chassisOrientation;

    uint8_t chassisOrientationName[3];
    memcpy(chassisOrientationName, CHASSIS_ORIENTATION_START_NAME, sizeof(chassisOrientationName));

    RefSerial::configGraphicGenerics(
        &chassisOrientationGraphics.graphicData[0],
        chassisOrientationName,
        Tx::AddGraphicOperation::ADD_GRAPHIC,
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
        Tx::AddGraphicOperation::ADD_GRAPHIC,
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

}  // namespace aruwsrc::display
