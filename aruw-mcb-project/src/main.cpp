#include <modm/board.hpp>
#include <modm/platform/core/delay.hpp>

int main()
{
    Board::initialize();

    while (true)
    {
        Board::Leds::toggle();
        modm::delayMilliseconds(1);
    }
}
