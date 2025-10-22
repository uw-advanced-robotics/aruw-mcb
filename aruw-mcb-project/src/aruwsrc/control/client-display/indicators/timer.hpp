#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{

class Timer : public HudIndicator, protected modm::Resumable<2>
{
public:
    Timer(
        tap::communication::serial::RefSerialTransmitter& refSerialTransmitter,
        const tap::communication::serial::RefSerial& refSerial);

    void initialize() override final;

    modm::ResumableResult<void> sendInitialGraphics() override final;

    modm::ResumableResult<void> update() override final;

private:
    static constexpr uint16_t TEXT_X = 611;
    static constexpr uint16_t NUMBER_X = 1018;
    // Y position of the text
    static constexpr uint16_t TEXT_Y = 200;
    // WIDTH of the text
    static constexpr uint16_t WIDTH = 4;

    static constexpr uint16_t SIZE = 80;

    Tx::GraphicCharacterMessage textGraphic;
    const char* text = "HELLO: ";
    Tx::Graphic1Message numberGraphic;
    tap::communication::referee::StateHUDIndicator<int32_t> numberIndicator;

    int timer = 0;
    const tap::communication::serial::RefSerial& refSerial;

    static inline void updateTimer(int32_t val, RefSerialData::Tx::Graphic1Message* graphic)
    {
        RefSerialTransmitter::configInteger(
            SIZE,
            WIDTH,
            TEXT_X,
            TEXT_Y,
            val,
            &graphic->graphicData

        );
    }
};
}  // namespace aruwsrc::control::client_display