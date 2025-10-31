#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "modm/processing/resumable.hpp"
#include "hud_indicator.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
    class Timer: public HudIndicator, modm::Resumable<2>
    {
        public:

            Timer(tap::communication::serial::RefSerialTransmitter& refSerialTransmitter,
                  const tap::communication::serial::RefSerial& refSerial);
            void initialize();
            modm::ResumableResult<void> sendInitialGraphics();
            modm::ResumableResult<void> update();
        

        private:
            static uint16_t TEXT_X = 100;
            static uint16_t NUMBER_X = 500;
            static uint16_t TEXT_Y = 100;
            static uint16_t WIDTH = 10;
            static uint16_t SIZE = 50;
            
            Tx::GraphicCharacterMessage textGraphic;
            const char* text = "Timer: ";

            Tx::Graphic1Message numberGraphic;
            tap::communication::referee::StateHUDIndicator<int32_t> numberIndicator;

            int timer = 0;
            const tap::communication::serial::RefSerial& refSerial;

            static void updateTimer(uint32_t value, RefSerialData::Tx::Graphic1Message* graphic)
            {
                RefSerialTransmitter::configInteger(
                    SIZE,
                    WIDTH,
                    TEXT_X,
                    TEXT_Y,
                    val,
                    &graphic -> graphicData
                );
            }
    }
}
