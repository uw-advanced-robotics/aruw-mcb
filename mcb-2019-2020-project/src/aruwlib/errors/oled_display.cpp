#include "oled_display.hpp"

namespace aruwlib
{

namespace errors
{

void OledDisplay::initialize()
{
    GpioOutputA6::setAnalogInput();
    Adc1::setPinChannel<GpioOutputA6>();

    buttonIsIdle = true;

    OledDisplay::display.initializeBlocking();
    OledDisplay::display.setFont(modm::font::ScriptoNarrow);

    modm::ViewStack vs(&display);
    menu = new ErrorMenu(&vs);
}

void OledDisplay::handleButtonStatus() {
    uint16_t buttonADC = Adc1::readChannel(Adc1::getPinChannel<GpioOutputA6>());
    if (buttonADC > 3500)
    {
        buttonIsIdle = true;
        return;
    }
    if (buttonIsIdle)
    {
        buttonIsIdle = false;
        if (buttonADC > 2600) {
            menu->shortButtonPress(modm::MenuButtons::DOWN);
        }
        else if (buttonADC > 2000)
        {
            menu->shortButtonPress(modm::MenuButtons::UP);
        }
        else if (buttonADC > 1000)
        {
            menu->shortButtonPress(modm::MenuButtons::RIGHT);
        }
        else if (buttonADC > 500)
        {
            menu->shortButtonPress(modm::MenuButtons::LEFT);
        }
        else
        {
            menu->shortButtonPress(modm::MenuButtons::OK);
        }
    }
}

void OledDisplay::update()
{
    handleButtonStatus();
    menu->update();
}

}  // namespace errors

}  // namespace aruwlib