#include "ErrorMenu.hpp"

namespace aruwlib
{
namespace display
{
ErrorMenu::ErrorMenu(modm::ViewStack *vs, Drivers *drivers)
    : AbstractMenu(vs, 1),
      viewStack(vs),
      display_update_time(500),
      menuName("Error Menu"),
      drivers(drivers)
{
}

void ErrorMenu::update()
{
    if (this->hasChanged())
    {
        this->draw();
    }
}

void ErrorMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    switch (button)
    {
        case modm::MenuButtons::LEFT:
            this->viewStack->pop();
            break;
        case modm::MenuButtons::RIGHT:
            break;
        case modm::MenuButtons::DOWN:
            break;
        case modm::MenuButtons::UP:
            break;
        case modm::MenuButtons::OK:
            break;
    }
}

bool ErrorMenu::hasChanged() { return true; }

void ErrorMenu::draw()
{
    modm::GraphicDisplay &display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << this->menuName;
}
}  // namespace display
}  // namespace aruwlib
