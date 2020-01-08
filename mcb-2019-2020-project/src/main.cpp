#include <rm-dev-board-a/board.hpp>

#include "src/communication/remote.hpp"
#include "src/communication/KeyToggling/KeyStateToggle.hpp"

bool keyA = false;
    KeyStateToggle toggle(aruwlib::Remote::Key::A);

int main()
{
    Board::initialize();
    aruwlib::Remote::initialize();


    while (1)
    {
        keyA = aruwlib::Remote::keyPressed(aruwlib::Remote::Key::A);
        aruwlib::Remote::read();
        toggle.KeyToggleHandler(false);
        Board::Leds::toggle();
        modm::delayMicroseconds(10);
    }
    return 0;
}
