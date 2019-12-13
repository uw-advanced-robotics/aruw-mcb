#include <rm-dev-board-a/board.hpp>
#include "src/aruwlib/communication/serial/dji_serial.hpp"

#include <modm/platform/uart/uart_2.hpp>

using namespace aruwlib::serial;

int main()
{
    Board::initialize();
    while (1)
    {
        modm::delayMicroseconds(10);
    }
    return 0;
}
