#include "timer_indicator.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
TimerIndicator::TimerIndicator(RefSerialTransmitter &refserialTransmitter)
    : HudIndicator(refserialTransmitter)
{}

void TimerIndicator::initialize() {
    uint8_t timerName[3];

    getUnusedGraphicName(timerName);

    RefSerialTransmitter::configGraphicGenerics(
        &timerGraphics.graphicData,
        timerName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::PINK);

    RefSerialTransmitter::configInteger(
        FONT_SIZE,
        WIDTH,
        TEXT_X,
        TEXT_Y,
        0,
        &timerGraphics.graphicData);

    startTime = tap::arch::clock::getTimeMilliseconds();
}

modm::ResumableResult<void> TimerIndicator::sendInitialGraphics() {
    RF_BEGIN(0);

    RF_CALL(refSerialTransmitter.sendGraphic(&timerGraphics));

    RF_END();
}

modm::ResumableResult<void> TimerIndicator::update() {
    uint32_t time = tap::arch::clock::getTimeMilliseconds() - startTime;
    
    RF_BEGIN(1);

    RefSerialTransmitter::configInteger(
        FONT_SIZE,
        WIDTH,
        TEXT_X,
        TEXT_Y,
        time / 1000,
        &timerGraphics.graphicData);

    RF_CALL(refSerialTransmitter.sendGraphic(&timerGraphics));

    RF_END();
}

}