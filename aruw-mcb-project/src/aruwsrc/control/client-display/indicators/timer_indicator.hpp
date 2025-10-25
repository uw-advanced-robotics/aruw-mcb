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
    TimerIndicator(tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);

    void initialize() override final;

    modm::ResumableResult<void> update() override final;

private:
    uint32_t initTime;  

    static constexpr uint32_t MILLISECONDS_TO_SECONDS = 1000; 

    static constexpr int16_t OFFSET_X = 0;
    static constexpr int16_t OFFSET_Y = 0;

    static constexpr uint16_t TIMER_X = SCREEN_WIDTH / 4 + OFFSET_X;
    static constexpr uint16_t TIMER_Y = SCREEN_HEIGHT / 4 + OFFSET_Y;

    static constexpr uint16_t TIMER_WIDTH = 10;
    static constexpr uint16_t FONT_THICKNESS = 3;

    Tx::Graphic1Message timerGraphics;
};

} // namespace aruwsrc::control::client_display

#endif // TIMER_INDICATOR_HPP_