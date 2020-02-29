#ifndef __OLED_MENU_HPP__
#define __OLED_MENU_HPP__

#include "modm/ui/menu/choice_menu.hpp"
#include "rm-dev-board-a/board.hpp"
#include "modm/ui/menu/menu_buttons.hpp"

namespace aruwlib
{

namespace errors
{

class OledMenu : public modm::ChoiceMenu
{
 public:
    explicit OledMenu(modm::ViewStack *vs);

    void draw() override;

    /**
     * @brief openNextScreen puts the next screen to be displayed on the stack
     */
    void openNextScreen() override;

 private:
    void handleButtonStatus();
    bool buttonIsIdle;
    modm::ViewStack *viewStack;
};

}  // namespace errors

}  // namespace aruwlib

#endif
