#ifndef __OLED_MENU_HPP__
#define __OLED_MENU_HPP__

#include "modm/ui/menu/abstract_menu.hpp"
#include "rm-dev-board-a/board.hpp"
#include "modm/ui/menu/menu_buttons.hpp"
#include "modm/processing/timer/periodic_timer.hpp"
#include <list>

namespace aruwlib
{

namespace errors
{

class OledMenu : public modm::AbstractMenu
{
 public:
    explicit OledMenu(modm::ViewStack *vs);

    void draw() override;

    void update() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    bool hasChanged() override;

    void addEntry(std::string name, std::string value);

 protected:
    struct ErrorMenuEntry {
        std::string name;
        std::string value;
    };

    std::list<ErrorMenuEntry> errorList;

    std::string title;

    int currentPosition = 0;

    int maxEntries = 17;

 private:
    void handleButtonStatus();
    bool buttonIsIdle;

    uint16_t display_update_time;
    modm::ShortPeriodicTimer timer;

    modm::ViewStack *viewStack;
};

}  // namespace errors

}  // namespace aruwlib

#endif
