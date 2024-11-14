#include "number_spam.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
NumberSpam::NumberSpam(RefSerialTransmitter &refSerialTransmitter)
    : HudIndicator(refSerialTransmitter)
{
}

modm::ResumableResult<bool> NumberSpam::update()
{
    RF_BEGIN(0);

    if (!delayTimeout.execute())
    {
        RF_RETURN(false);
    }
    delayTimeout.restart(timeout);

    time = tap::arch::clock::getTimeMilliseconds();

    // Draw the number
    numberGraphic.graphicData.operation = numberGraphic.graphicData.operation == Tx::GRAPHIC_DELETE
                                              ? Tx::GRAPHIC_ADD
                                              : Tx::GRAPHIC_MODIFY;

    RefSerialTransmitter::configInteger(20, 4, TEXT_X, TEXT_Y, time, &numberGraphic.graphicData);

    RF_CALL(refSerialTransmitter.sendGraphic(&numberGraphic));

    RF_END();
}

modm::ResumableResult<bool> NumberSpam::sendInitialGraphics()
{
    RF_BEGIN(1);

    delayTimeout.restart(timeout);

    RF_CALL(refSerialTransmitter.sendGraphic(&numberGraphic));

    RF_END();
}

void NumberSpam::initialize()
{
    uint8_t graphicName[3];

    getUnusedGraphicName(graphicName);
    RefSerialTransmitter::configGraphicGenerics(
        &numberGraphic.graphicData,
        graphicName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::ORANGE);

    RefSerialTransmitter::configInteger(20, 4, TEXT_X, TEXT_Y, 100, &numberGraphic.graphicData);
}

}  // namespace aruwsrc::control::client_display
