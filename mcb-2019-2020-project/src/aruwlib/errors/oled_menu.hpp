#ifndef __OLED_MENU_HPP__
#define __OLED_MENU_HPP__

#include "modm/ui/menu/choice_menu.hpp"

namespace aruwlib
{

namespace errors
{

class OledMenu : public modm::ChoiceMenu
{
 public:

    void init();

    /**
     * @brief openNextScreen puts the next screen to be displayed on the stack
     */
    void openNextScreen() override;

protected:

    //typedef modm::DoublyLinkedList<ChoiceMenuEntry> EntryList;
    //EntryList entries;
};

}  // namespace errors

}  // namespace aruwlib

#endif
