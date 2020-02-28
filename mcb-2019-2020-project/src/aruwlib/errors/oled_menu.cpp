#include "oled_menu.hpp"
#include "src/aruwlib/display/sh1106.hpp"

namespace aruwlib 
{

namespace errors
{
    modm::MenuButtons::Button getButtonStatus();

    OledMenu::OledMenu(modm::ViewStack *vs) : ChoiceMenu(vs, 1, "Title"), viewStack(vs) {
        initialise();
        GpioOutputA6::setAnalogInput();
        Adc1::setPinChannel<GpioOutputA6>();
    }

    void OledMenu::openNextScreen() {
        
    }

    void OledMenu::draw() {
        ChoiceMenu::draw();
        getButtonStatus();
        //modm::MenuButtons::Button currButton(getButtonStatus());
        // if (currButton != IDLE) {
        //     ChoiceMenu::shortButtonPress(currButton);
        // }
    }

    uint16_t bruh;
    void OledMenu::getButtonStatus() {
        bruh = Adc1::readChannel(Adc1::getPinChannel<GpioOutputA6>());
    }
}

}
