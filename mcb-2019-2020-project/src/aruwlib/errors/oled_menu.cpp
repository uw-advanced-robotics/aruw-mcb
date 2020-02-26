#include "oled_menu.hpp"

namespace aruwlib 
{

namespace errors
{
    void OledMenu::init() {
        ChoiceMenu(new modm::ViewStack(new modm::GraphicDisplay()), 1);
        OledMenu::initialise();
    }
}

}
