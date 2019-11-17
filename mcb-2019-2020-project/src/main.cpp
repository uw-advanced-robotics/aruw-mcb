#include <rm-dev-board-a/board.hpp>
#include "src/sensors/ir/analog_ir.hpp"

float distance;

int main()
{
    Board::initialize();

    aruwlib::sensors::AnalogIR sharpIR(4, 30);
    sharpIR.init();

    while (1)
    {
        distance = sharpIR.read();
    }
    return 0;
}
