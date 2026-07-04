#include "pump_indicator.hpp"

#include "tap/architecture/clock.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display::indicators
{
PumpIndicator::PumpIndicator(
    tap::communication::serial::RefSerialTransmitter& refSerialTransmitter,
    aruwsrc::control::digital::DigitalOutSubsystem& subsystem)
    : HudIndicator(refSerialTransmitter),
      subsystem(subsystem)
{
}

modm::ResumableResult<void> PumpIndicator::update()
{
    uint32_t prevOperation = -1;

    RF_BEGIN(1);

    prevOperation = pumpGraphic.graphicData.operation;

    if (subsystem.getState())
    {
        pumpGraphic.graphicData.operation =
            prevOperation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD : Tx::GRAPHIC_MODIFY;
    }
    else
    {
        pumpGraphic.graphicData.operation = Tx::GRAPHIC_DELETE;
    }

    // Don't resend if it's already deleted and staying deleted
    if (prevOperation == Tx::GRAPHIC_DELETE &&
        pumpGraphic.graphicData.operation == Tx::GRAPHIC_DELETE)
    {
        RF_RETURN();
    }

    RefSerialTransmitter::configCircle(
        PUMP_INDICATOR_RADIUS,
        X_POS,
        Y_POS,
        PUMP_INDICATOR_RADIUS,
        &pumpGraphic.graphicData);

    RF_CALL(refSerialTransmitter.sendGraphic(&pumpGraphic));

    RF_END();
}

void PumpIndicator::initialize()
{
    uint8_t indicatorName[3];

    getUnusedGraphicName(indicatorName);
    RefSerialTransmitter::configGraphicGenerics(
        &pumpGraphic.graphicData,
        indicatorName,
        Tx::GRAPHIC_DELETE,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::GREEN);  // pick whichever color distinguishes this from other indicators
}

}  // namespace aruwsrc::control::client_display::indicators