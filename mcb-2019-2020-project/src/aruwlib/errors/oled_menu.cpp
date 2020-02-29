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
        handleButtonStatus();
    }

    void OledMenu::handleButtonStatus() {
        uint16_t buttonADC= Adc1::readChannel(Adc1::getPinChannel<GpioOutputA6>());
        if (buttonADC > 4000) {
            buttonIsIdle = true;
            return;
        }
        if (buttonIsIdle) {
            buttonIsIdle = false;
            if (buttonADC > 3000) {
                ChoiceMenu::shortButtonPress(modm::MenuButtons::DOWN);
            }
            else if (buttonADC > 2000) {
                ChoiceMenu::shortButtonPress(modm::MenuButtons::UP);
            }
            else if (buttonADC > 1000) {
                ChoiceMenu::shortButtonPress(modm::MenuButtons::RIGHT);
            }   
            else if (buttonADC > 500) {
                ChoiceMenu::shortButtonPress(modm::MenuButtons::LEFT);
            }
            ChoiceMenu::shortButtonPress(modm::MenuButtons::OK);
        }
    }
}

}
