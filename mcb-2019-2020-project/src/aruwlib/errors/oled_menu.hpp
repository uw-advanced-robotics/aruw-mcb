#ifndef __OLED_MENU_HPP__
#define __OLED_MENU_HPP__

#include "modm/ui/menu/choice_menu.hpp"
#include "rm-dev-board-a/board.hpp"

namespace aruwlib
{

namespace errors
{

class OledMenu : public modm::ChoiceMenu
{
 public:

    OledMenu(modm::ViewStack *vs);

    /**
     * @brief openNextScreen puts the next screen to be displayed on the stack
     */
    void openNextScreen() override;

 protected:

    //typedef modm::DoublyLinkedList<ChoiceMenuEntry> EntryList;
    //EntryList entries;
 private:
    modm::ViewStack *viewStack;
};

}  // namespace errors

}  // namespace aruwlib

#endif
