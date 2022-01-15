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
#if defined(TARGET_HERO)
      hudIndicatorDrawers{
          BooleanDrawer(drivers, &hudIndicatorGraphics[0]),
          BooleanDrawer(drivers, &hudIndicatorGraphics[1]),
          BooleanDrawer(drivers, &hudIndicatorGraphics[2]),
          BooleanDrawer(drivers, &hudIndicatorGraphics[3]),
          BooleanDrawer(drivers, &hudIndicatorGraphics[4]),
      },
#else
      hudIndicatorDrawers{
          BooleanDrawer(drivers, &hudIndicatorGraphics[0]),
          BooleanDrawer(drivers, &hudIndicatorGraphics[1]),
          BooleanDrawer(drivers, &hudIndicatorGraphics[2]),
          BooleanDrawer(drivers, &hudIndicatorGraphics[3]),
      },
#endif
      driveCommands{driveCommands},
      turretSubsystem(turretSubsystem)
{
    addSubsystemRequirement(clientDisplay);
    hudIndicatorDrawers[AGITATOR_STATUS_HEALTHY].setBoolFalseColor(Tx::GraphicColor::PURPLISH_RED);
}

void ClientDisplayCommand::initialize()
{
    restart();  // restart protothread
    initializeHudIndicators();
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
        updateHudIndicators();

        for (hudIndicatorIndex = 0; hudIndicatorIndex < NUM_HUD_INDICATORS; hudIndicatorIndex++)
        {
            PT_CALL(hudIndicatorDrawers[hudIndicatorIndex].draw());
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

    for (hudIndicatorIndex = 0; hudIndicatorIndex < NUM_HUD_INDICATORS; hudIndicatorIndex++)
    {
        RF_CALL(hudIndicatorDrawers[hudIndicatorIndex].initialize());

        drivers->refSerial.sendGraphic(&hudIndicatorStaticGraphics[hudIndicatorIndex]);
        delay(&hudIndicatorStaticGraphics[hudIndicatorIndex]);
        drivers->refSerial.sendGraphic(&hudIndicatorStaticLabelGraphics[hudIndicatorIndex]);
        delay(&hudIndicatorStaticLabelGraphics[hudIndicatorIndex]);
    }

    for (reticleIndex = 0; reticleIndex < MODM_ARRAY_SIZE(reticleMsg); reticleIndex++)
    {
        drivers->refSerial.sendGraphic(&reticleMsg[reticleIndex]);
        delay(&reticleMsg[reticleIndex]);
    }

    drivers->refSerial.sendGraphic(&driveCommandMsg);
    delay(&driveCommandMsg);

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

        delay(&driveCommandMsg);
    }

    // No delay necessary since didn't send anything
    RF_END();
}

modm::ResumableResult<bool> ClientDisplayCommand::updateVehicleOrientation()
{
    RF_BEGIN(2)

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

void ClientDisplayCommand::updateHudIndicators()
{
    if (hopperSubsystem != nullptr)
    {
        hudIndicatorDrawers[HOPPER_OPEN].setDrawerColor(!hopperSubsystem->getIsHopperOpen());
    }

    if (frictionWheelSubsystem != nullptr)
    {
        hudIndicatorDrawers[FRICTION_WHEELS_ON].setDrawerColor(
            !compareFloatClose(0.0f, frictionWheelSubsystem->getDesiredLaunchSpeed(), 1E-5));
    }

    hudIndicatorDrawers[CV_AIM_DATA_VALID].setDrawerColor(
        drivers->legacyVisionCoprocessor.lastAimDataValid());

    if (agitatorSubsystem != nullptr)
    {
        hudIndicatorDrawers[AGITATOR_STATUS_HEALTHY].setDrawerColor(!agitatorSubsystem->isJammed());
    }
}

void ClientDisplayCommand::initializeHudIndicators()
{
    uint8_t hudIndicatorName[3];
    memcpy(hudIndicatorName, HUD_INDICATOR_LIST_START_NAME, sizeof(hudIndicatorName));
    uint16_t hudIndicatorListCurrY = HUD_INDICATOR_LIST_START_Y;

    // Configure hopper cover hud indicator
    for (hudIndicatorIndex = 0; hudIndicatorIndex < NUM_HUD_INDICATORS; hudIndicatorIndex++)
    {
        RefSerial::configGraphicGenerics(
            &hudIndicatorGraphics[hudIndicatorIndex].graphicData,
            hudIndicatorName,
            Tx::ADD_GRAPHIC,
            HUD_INDICATOR_LIST_LAYER,
            HUD_INDICATOR_FILLED_COLOR);

        RefSerial::configCircle(
            HUD_INDICATOR_WIDTH,
            HUD_INDICATOR_LIST_CENTER_X,
            hudIndicatorListCurrY,
            HUD_INDICATOR_RADIUS,
            &hudIndicatorGraphics[hudIndicatorIndex].graphicData);

        hudIndicatorName[2]++;

        RefSerial::configGraphicGenerics(
            &hudIndicatorStaticGraphics[hudIndicatorIndex].graphicData,
            hudIndicatorName,
            Tx::ADD_GRAPHIC,
            HUD_INDICATOR_STATIC_LIST_LAYER,
            HUD_INDICATOR_OUTLINE_COLOR);

        RefSerial::configCircle(
            HUD_INDICATOR_OUTLINE_WIDTH,
            HUD_INDICATOR_LIST_CENTER_X,
            hudIndicatorListCurrY,
            HUD_INDICATOR_OUTLINE_RADIUS,
            &hudIndicatorStaticGraphics[hudIndicatorIndex].graphicData);

        hudIndicatorName[2]++;

        RefSerial::configGraphicGenerics(
            &hudIndicatorStaticLabelGraphics[hudIndicatorIndex].graphicData,
            hudIndicatorName,
            Tx::ADD_GRAPHIC,
            HUD_INDICATOR_STATIC_LIST_LAYER,
            HUD_INDICATOR_LABEL_COLOR);

        RefSerial::configCharacterMsg(
            HUD_INDICATOR_LABEL_CHAR_SIZE,
            HUD_INDICATOR_LABEL_CHAR_LENGTH,
            HUD_INDICATOR_LABEL_CHAR_LINE_WIDTH,
            HUD_INDICATOR_LIST_CENTER_X -
                strlen(HUD_INDICATOR_LABELS[hudIndicatorIndex]) * HUD_INDICATOR_LABEL_CHAR_SIZE -
                HUD_INDICATOR_OUTLINE_RADIUS - HUD_INDICATOR_OUTLINE_WIDTH / 2,
            hudIndicatorListCurrY + HUD_INDICATOR_LABEL_CHAR_SIZE / 2,
            HUD_INDICATOR_LABELS[hudIndicatorIndex],
            &hudIndicatorStaticLabelGraphics[hudIndicatorIndex]);

        hudIndicatorName[2]++;

        hudIndicatorListCurrY += HUD_INDICATOR_LIST_DIST_BTWN_BULLETS;
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
