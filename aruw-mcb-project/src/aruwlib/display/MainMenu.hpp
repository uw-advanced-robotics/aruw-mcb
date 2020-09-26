#ifndef MAIN_MENU_HPP_
#define MAIN_MENU_HPP_

#include <modm/ui/menu/standard_menu.hpp>

namespace aruwlib
{
class Drivers;
namespace display
{
class MainMenu : public modm::StandardMenu
{
public:
    MainMenu(modm::ViewStack *stack, uint8_t identifier, Drivers *drivers);

    virtual ~MainMenu() = default;

    void initialize();

    void addErrorMenuCallback();
    void addMotorMenuCallback();
    void addPropertyTableCallback();

private:
    Drivers *drivers;
};  // class MainMenu
}  // namespace display
}  // namespace aruwlib

#endif  // MAIN_MENU_HPP_
