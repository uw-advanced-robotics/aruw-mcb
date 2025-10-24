#include "timer.hpp"

#include "tap/drivers.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
long startTime;
Timer::Timer(RefSerialTransmitter& refSerialTransmitter, const RefSerial& refSerial)
    : HudIndicator(refSerialTransmitter),
      numberIndicator(refSerialTransmitter, &numberGraphic, updateTimer, 0),
      refSerial(refSerial)
{
    startTime = tap::arch::clock::getTimeMilliseconds() / 1000;
}

modm::ResumableResult<void> Timer::update()
{
    timer = (tap::arch::clock::getTimeMilliseconds() / 1000) - startTime;

    RF_BEGIN(1);

    numberIndicator.setIndicatorState(timer);

    RF_END();
    RF_CALL(numberIndicator.draw());
}

modm::ResumableResult<void> Timer::sendInitialGraphics()
{
    RF_BEGIN(0);

    RF_CALL(refSerialTransmitter.sendGraphic(&textGraphic));

    RF_CALL(numberIndicator.initialize());

    RF_END();
}

void Timer::initialize()
{
    uint8_t graphicName[3];

    getUnusedGraphicName(graphicName);
    RefSerialTransmitter::configGraphicGenerics(
        &textGraphic.graphicData,
        graphicName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::GREEN);

    RefSerialTransmitter::configCharacterMsg(SIZE, WIDTH, TEXT_X, TEXT_Y, "TIMER: ", &textGraphic);

    getUnusedGraphicName(graphicName);
    RefSerialTransmitter::configGraphicGenerics(
        &numberGraphic.graphicData,
        graphicName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::YELLOW);

        updateTimer(0, &numberGraphic);
}
}  // namespace aruwsrc::control::client_display