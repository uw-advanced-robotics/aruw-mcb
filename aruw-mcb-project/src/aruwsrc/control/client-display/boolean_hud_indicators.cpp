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

#include "boolean_hud_indicators.hpp"

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"

#include "tap/drivers.hpp"

using namespace tap::communication::serial;
using namespace tap::communication::referee;

namespace aruwsrc::control::client_display
{
/**
 * Updates the color of a graphic to be either `ON_COLOR` if `indicatorStatus == true` or
 * `OFF_COLOR` if `indicatorStatus == false`.
 */
template <RefSerial::Tx::GraphicColor ON_COLOR, RefSerial::Tx::GraphicColor OFF_COLOR>
static inline void updateGraphicColor(
    bool indicatorStatus,
    RefSerialData::Tx::Graphic1Message *graphic)
{
    graphic->graphicData.color =
        static_cast<uint32_t>(indicatorStatus ? ON_COLOR : OFF_COLOR) & 0xf;
}

BooleanHudIndicators::BooleanHudIndicators(
    tap::Drivers &drivers,
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
    const aruwsrc::control::TurretMCBHopperSubsystem *hopperSubsystem,
    const aruwsrc::control::launcher::FrictionWheelSubsystem &frictionWheelSubsystem,
    tap::control::setpoint::SetpointSubsystem &agitatorSubsystem,
    const aruwsrc::control::imu::ImuCalibrateCommand &imuCalibrateCommand,
    const aruwsrc::communication::serial::SentryResponseHandler &sentryResponseHandler)
    : HudIndicator(refSerialTransmitter),
      drivers(drivers),
      hopperSubsystem(hopperSubsystem),
      frictionWheelSubsystem(frictionWheelSubsystem),
      agitatorSubsystem(agitatorSubsystem),
      imuCalibrateCommand(imuCalibrateCommand),
      sentryResponseHandler(sentryResponseHandler),
      booleanHudIndicatorDrawers{
          BooleanHUDIndicator(
              refSerialTransmitter,
              &booleanHudIndicatorGraphics[SYSTEMS_CALIBRATING],
              updateGraphicColor<
                  std::get<1>(BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[SYSTEMS_CALIBRATING]),
                  std::get<2>(BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[SYSTEMS_CALIBRATING])>,
              0),
          BooleanHUDIndicator(
              refSerialTransmitter,
              &booleanHudIndicatorGraphics[AGITATOR_STATUS_HEALTHY],
              updateGraphicColor<
                  std::get<1>(BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[AGITATOR_STATUS_HEALTHY]),
                  std::get<2>(BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[AGITATOR_STATUS_HEALTHY])>,
              0),
          BooleanHUDIndicator(
              refSerialTransmitter,
              &booleanHudIndicatorGraphics[SENTRY_DRIVE_STATUS],
              updateGraphicColor<
                  std::get<1>(BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[SENTRY_DRIVE_STATUS]),
                  std::get<2>(BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[SENTRY_DRIVE_STATUS])>,
              0),
      }
{
}

modm::ResumableResult<bool> BooleanHudIndicators::sendInitialGraphics()
{
    RF_BEGIN(0);

    // send all boolean hud indicator graphics (labels and circles)
    for (booleanHudIndicatorIndexSendInitialGraphics = 0;
         booleanHudIndicatorIndexSendInitialGraphics < NUM_BOOLEAN_HUD_INDICATORS;
         booleanHudIndicatorIndexSendInitialGraphics++)
    {
        RF_CALL(
            booleanHudIndicatorDrawers[booleanHudIndicatorIndexSendInitialGraphics].initialize());

        RF_CALL(refSerialTransmitter.sendGraphic(
            &booleanHudIndicatorStaticGraphics[booleanHudIndicatorIndexSendInitialGraphics]));
        RF_CALL(refSerialTransmitter.sendGraphic(
            &booleanHudIndicatorStaticLabelGraphics[booleanHudIndicatorIndexSendInitialGraphics]));
    }

    RF_END();
}

modm::ResumableResult<bool> BooleanHudIndicators::update()
{
    RF_BEGIN(1);

    // update agitator state
    booleanHudIndicatorDrawers[AGITATOR_STATUS_HEALTHY].setIndicatorState(
        agitatorSubsystem.isOnline() && !agitatorSubsystem.isJammed());

    booleanHudIndicatorDrawers[SYSTEMS_CALIBRATING].setIndicatorState(
        drivers.commandScheduler.isCommandScheduled(&imuCalibrateCommand));

    booleanHudIndicatorDrawers[SENTRY_DRIVE_STATUS].setIndicatorState(
        sentryResponseHandler.getSentryMoving());

    // draw all the booleanHudIndicatorDrawers (only actually sends data if graphic changed)
    for (booleanHudIndicatorIndexUpdate = 0;
         booleanHudIndicatorIndexUpdate < NUM_BOOLEAN_HUD_INDICATORS;
         booleanHudIndicatorIndexUpdate++)
    {
        RF_CALL(booleanHudIndicatorDrawers[booleanHudIndicatorIndexUpdate].draw());
    }

    RF_END();
}

void BooleanHudIndicators::initialize()
{
    uint8_t booleanHudIndicatorName[3] = {};
    uint16_t hudIndicatorListCurrY = BOOLEAN_HUD_INDICATOR_LIST_START_Y;

    // Configure hopper cover hud indicator
    for (int i = 0; i < NUM_BOOLEAN_HUD_INDICATORS; i++)
    {
        // config the boolean HUD indicator circle (that will switch colors based on state)
        getUnusedGraphicName(booleanHudIndicatorName);

        RefSerialTransmitter::configGraphicGenerics(
            &booleanHudIndicatorGraphics[i].graphicData,
            booleanHudIndicatorName,
            Tx::GRAPHIC_ADD,
            DEFAULT_GRAPHIC_LAYER,
            std::get<2>(BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[i]));

        RefSerialTransmitter::configCircle(
            BOOLEAN_HUD_INDICATOR_WIDTH,
            BOOLEAN_HUD_INDICATOR_LIST_CENTER_X,
            hudIndicatorListCurrY,
            BOOLEAN_HUD_INDICATOR_RADIUS,
            &booleanHudIndicatorGraphics[i].graphicData);

        // config the border circle that bounds the booleanHudIndicatorGraphics
        getUnusedGraphicName(booleanHudIndicatorName);

        RefSerialTransmitter::configGraphicGenerics(
            &booleanHudIndicatorStaticGraphics[i].graphicData,
            booleanHudIndicatorName,
            Tx::GRAPHIC_ADD,
            DEFAULT_GRAPHIC_LAYER,
            BOOLEAN_HUD_INDICATOR_OUTLINE_COLOR);

        RefSerialTransmitter::configCircle(
            BOOLEAN_HUD_INDICATOR_OUTLINE_WIDTH,
            BOOLEAN_HUD_INDICATOR_LIST_CENTER_X,
            hudIndicatorListCurrY,
            BOOLEAN_HUD_INDICATOR_OUTLINE_RADIUS,
            &booleanHudIndicatorStaticGraphics[i].graphicData);

        // config the label associated with the particular indicator
        getUnusedGraphicName(booleanHudIndicatorName);

        RefSerialTransmitter::configGraphicGenerics(
            &booleanHudIndicatorStaticLabelGraphics[i].graphicData,
            booleanHudIndicatorName,
            Tx::GRAPHIC_ADD,
            DEFAULT_GRAPHIC_LAYER,
            BOOLEAN_HUD_INDICATOR_LABEL_COLOR);

        const char *indicatorLabel = std::get<0>(BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[i]);

        RefSerialTransmitter::configCharacterMsg(
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
}  // namespace aruwsrc::control::client_display
