#ifndef ERROR_MENU_HPP_
#define ERROR_MENU_HPP_

#include <list>

#include <modm/ui/menu/abstract_menu.hpp>

#include "aruwlib/rm-dev-board-a/board.hpp"

#include "modm/processing/timer/periodic_timer.hpp"

namespace aruwlib
{
class Drivers;
namespace display
{
class ErrorMenu : public modm::AbstractMenu
{
public:
    ErrorMenu(modm::ViewStack *vs, Drivers *drivers);

    void draw() override;

    void update() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    bool hasChanged() override;

    const char *getMenuName() const { return menuName; }

private:
    modm::ViewStack *viewStack;
    uint16_t display_update_time;
    const char *menuName;
    Drivers *drivers;
};  // class ErrorMenu
}  // namespace display
}  // namespace aruwlib

#endif  // ERROR_MENU_HPP_
