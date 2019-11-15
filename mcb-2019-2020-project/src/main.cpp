#include <rm-dev-board-a/board.hpp>
#include "src/sensors/ir/analog_ir.hpp"

float distance;

int main()
{
    Board::initialize();

    aruwlib::sensors::AnalogIR test(0, 30, 137500, 1125, aruwlib::gpio::Analog::Pin::U);
    test.init();

    while (1)
    {
        //Board::Leds::toggle();
        //modm::delayMilliseconds(1000);
        distance = test.read();
    }
    return 0;
}
