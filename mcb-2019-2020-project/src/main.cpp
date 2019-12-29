#include <rm-dev-board-a/board.hpp>
#include "src/aruwlib/communication/serial/dji_serial.hpp"

#include <modm/platform/uart/uart_2.hpp>

#include "src/aruwlib/communication/serial/ref_serial.hpp"

using namespace aruwlib::serial;

RefSerial refSerial;

int main()
{
    Board::initialize();

    refSerial.initialize();

    RefSerial::DisplayData displayData;
    displayData.bool1 = true;
    displayData.bool3 = true;
    displayData.bool2 = false;
    displayData.float1 = 54.0f;

    while (1)
    {
        refSerial.updateSerial();
        refSerial.sendDisplayData(displayData);
        modm::delayMicroseconds(10);
    }
    return 0;
}
