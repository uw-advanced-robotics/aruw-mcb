#include "timer_indicator.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
TimerIndicator::TimerIndicator(RefSerialTransmitter &refSerialTransmitter) : HudIndicator(refSerialTransmitter) {}

void TimerIndicator::initialize() {
    uint8_t indicatorName[3];

    getUnusedGraphicName(indicatorName);

    initTime = tap::arch::clock::getTimeMilliseconds();

    RefSerialTransmitter::configGraphicGenerics(
        &timerGraphics.graphicData,
        indicatorName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::GREEN);
}

modm::ResumableResult<void> TimerIndicator::update() {
    RF_BEGIN(0)

    uint32_t currentTime = tap::arch::clock::getTimeMilliseconds() - initTime;
    currentTime /= MILLISECONDS_TO_SECONDS;

    RefSerialTransmitter::configInteger(
        FONT_THICKNESS,
        TIMER_WIDTH,
        TIMER_X,
        TIMER_Y,
        currentTime,
        &timerGraphics.graphicData);

    RF_CALL(refSerialTransmitter.sendGraphic(&timerGraphics));

    RF_END();
}

}  // namespace aruwsrc::control::client_display