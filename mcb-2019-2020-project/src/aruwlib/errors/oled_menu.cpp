#include "oled_menu.hpp"
#include "src/aruwlib/display/sh1106.hpp"

namespace aruwlib 
{

namespace errors
{
    OledMenu::OledMenu(modm::ViewStack *vs) : ChoiceMenu(vs, 1, "Title"), viewStack(vs) {
        initialise();
    }

    void OledMenu::openNextScreen() {
       
    }
}

}
