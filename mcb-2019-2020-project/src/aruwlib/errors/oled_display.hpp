#ifndef __OLED_DISPLAY_HPP__
#define __OLED_DISPLAY_HPP__

#include "modm/ui/menu/view_stack.hpp"
#include "rm-dev-board-a/board.hpp"
#include "error_menu.hpp"
#include "src/aruwlib/display/sh1106.hpp"

namespace aruwlib
{

namespace errors
{

class OledDisplay
{
 public:
    void initialize();
    void update();
 private:
    void handleButtonStatus();
    bool buttonIsIdle;

    display::Sh1106<
        Board::DisplaySpiMaster,
        Board::DisplayCommand,
        Board::DisplayReset,
        128, 64,
        false
    > display;

    ErrorMenu *menu;
};

}  // namespace errors

}  // namespace aruwlib

#endif
