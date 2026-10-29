#ifndef TIMER_INDICATOR_HPP_
#define TIMER_INDICATOR_HPP_

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

namespace aruwsrc::control::client_display
{
class TimerIndicator : public HudIndicator, protected modm::Resumable<2>
{
public:
    /**
     * Makes a timer on the screen that increments its count every second.
     *
     * @param[in] refSerialTransmitter RefSerialTransmitter instance.
     */
    TimerIndicator(tap::communication::serial::RefSerialTransmitter& refSerialTransmitter);

    void initialize() override final;

    modm::ResumableResult<void> sendInitialGraphics() override final;

    modm::ResumableResult<void> update() override final;

private:
    static constexpr uint16_t FONT_SIZE = 16;
    static constexpr uint16_t WIDTH = 10;
    static constexpr uint16_t TEXT_X = SCREEN_WIDTH / 4;
    static constexpr uint16_t TEXT_Y = SCREEN_HEIGHT / 4;

    Tx::Graphic1Message timerGraphics;
    uint32_t startTime;
};
}

#endif