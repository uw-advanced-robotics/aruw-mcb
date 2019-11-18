#include <rm-dev-board-a/board.hpp>

#include "src/communication/sensors/mpu6500.hpp"
#include "src/communication/sensors/ist8310.hpp"

using namespace aruwlib::sensors;
int16_t istmx = 0;
int16_t istmy = 0;
int16_t istmz = 0;
int count = 0;

int main()
{
    Board::initialize();

    Mpu6500::init();
    Ist8310::init();

    while (1)
    {
        count++;
        Ist8310::ist8310GetData();
        istmx = Ist8310::ist.mx;
        istmy = Ist8310::ist.my;
        istmz = Ist8310::ist.mz;
        Board::Leds::toggle();
        modm::delayMilliseconds(2);
    }
    return 0;
}
