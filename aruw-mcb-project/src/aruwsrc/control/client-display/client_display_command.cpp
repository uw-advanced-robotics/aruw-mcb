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
          BooleanDrawer(drivers, &hudIndicatorGraphics[5]),
      },
#else
      hudIndicatorDrawers{
          BooleanDrawer(drivers, &hudIndicatorGraphics[0]),
          BooleanDrawer(drivers, &hudIndicatorGraphics[1]),
          BooleanDrawer(drivers, &hudIndicatorGraphics[2]),
          BooleanDrawer(drivers, &hudIndicatorGraphics[3]),
          BooleanDrawer(drivers, &hudIndicatorGraphics[4]),
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
    initializeChassisOrientation();
}

void ClientDisplayCommand::execute() { run(); }

bool ClientDisplayCommand::run()
{
    PT_BEGIN();

    PT_CALL(initializeNonblocking());

    while (true)
    {
        PT_CALL(updateHudIndicators());

        PT_CALL(updateDriveCommandMsg());

        PT_CALL(updateChassisOrientation());

        PT_CALL(updateTurretAngles());

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
    delay(&chassisOrientationGraphics);
    chassisOrientationGraphics.graphicData[0].operation = Tx::ADD_GRAPHIC_MODIFY;
    chassisOrientationGraphics.graphicData[1].operation = Tx::ADD_GRAPHIC_MODIFY;

    drivers->refSerial.sendGraphic(&turretAnglesGraphics);
    delay(&turretAnglesGraphics);
    turretAnglesGraphics.graphicData[0].operation = Tx::ADD_GRAPHIC_MODIFY;
    turretAnglesGraphics.graphicData[1].operation = Tx::ADD_GRAPHIC_MODIFY;

    drivers->refSerial.sendGraphic(&turretAnglesLabelGraphics);
    delay(&turretAnglesLabelGraphics);

    RF_END();
}

modm::ResumableResult<bool> ClientDisplayCommand::updateHudIndicators()
{
    RF_BEGIN(1);

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

    // TODO add hero fire ready and systems calibrating when available

    for (hudIndicatorIndex = 0; hudIndicatorIndex < NUM_HUD_INDICATORS; hudIndicatorIndex++)
    {
        RF_CALL(hudIndicatorDrawers[hudIndicatorIndex].draw());
    }

    RF_END();
}

modm::ResumableResult<bool> ClientDisplayCommand::updateDriveCommandMsg()
{
    RF_BEGIN(2);

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

modm::ResumableResult<bool> ClientDisplayCommand::updateChassisOrientation()
{
    RF_BEGIN(3)

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
    RF_BEGIN(4);

    if (turretSubsystem != nullptr)
    {
        yaw = turretSubsystem->getCurrentYawValue().getValue();
        pitch = turretSubsystem->getCurrentPitchValue().getValue();

        if (!compareFloatClose(
                prevYaw,
                yaw,
                1.0f / modm::pow(10, TURRET_ANGLES_DECIMAL_PRECISION)) ||
            !compareFloatClose(
                prevPitch,
                pitch,
                1.0f / modm::pow(10.0f, TURRET_ANGLES_DECIMAL_PRECISION)))
        {
            turretAnglesGraphics.graphicData[0].value = 1000.0f * yaw;
            turretAnglesGraphics.graphicData[1].value = 1000.0f * pitch;

            drivers->refSerial.sendGraphic(&turretAnglesGraphics);
            delay(&turretAnglesGraphics);

            prevYaw = yaw;
            prevPitch = pitch;
        }
    }

    RF_END();
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
        DRIVE_COMMAND_CHAR_LENGTH,
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
            &turretAnglesGraphics.graphicData[0],
            turretAnglesName,
            Tx::ADD_GRAPHIC,
            TURRET_ANGLES_LAYER,
            TURRET_ANGLES_COLOR);

        RefSerial::configFloatingNumber(
            TURRET_ANGLES_FONT_SIZE,
            TURRET_ANGLES_DECIMAL_PRECISION,
            TURRET_ANGLES_WIDTH,
            TURRET_ANGLES_START_X,
            TURRET_ANGLES_START_Y,
            0,
            &turretAnglesGraphics.graphicData[0]);

        turretAnglesName[0]++;

        RefSerial::configGraphicGenerics(
            &turretAnglesGraphics.graphicData[0],
            turretAnglesName,
            Tx::ADD_GRAPHIC,
            TURRET_ANGLES_LAYER,
            TURRET_ANGLES_COLOR);

        RefSerial::configFloatingNumber(
            TURRET_ANGLES_FONT_SIZE,
            TURRET_ANGLES_DECIMAL_PRECISION,
            TURRET_ANGLES_WIDTH,
            TURRET_ANGLES_START_X,
            TURRET_ANGLES_START_Y + TURRET_ANGLES_FONT_SIZE,
            0,
            &turretAnglesGraphics.graphicData[1]);

        RefSerial::configGraphicGenerics(
            &turretAnglesLabelGraphics.graphicData,
            turretAnglesName,
            Tx::ADD_GRAPHIC,
            TURRET_ANGLES_LAYER,
            TURRET_ANGLES_COLOR);

        RefSerial::configCharacterMsg(
            TURRET_ANGLES_FONT_SIZE,
            1,  // TODO check this
            TURRET_ANGLES_WIDTH,
            TURRET_ANGLES_START_X - strlen("PITCH") * TURRET_ANGLES_FONT_SIZE,
            TURRET_ANGLES_START_Y,
            "PITCH\nYAW  ",
            &turretAnglesLabelGraphics);

        prevYaw = 0.0f;
        prevPitch = 0.0f;
    }
}

}  // namespace aruwsrc::display
