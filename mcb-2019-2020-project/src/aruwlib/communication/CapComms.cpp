#include "CapComms.hpp"

#include "aruwlib/Drivers.hpp"
#include "aruwlib/rm-dev-board-a/board.hpp"

using namespace Board;

namespace aruwlib
{
namespace communication
{
void CapComms::initialize()
{
    Drivers::spi.initialize<Board::SystemClock, 703125_Hz>(Spi::SpiPort::SPI_4);
}

void CapComms::update()
{
    // todo
    // Drivers::spi.writeRegister(Spi::SpiPort::SPI_4, 0x1, 0xff);
}
}  // namespace communication
}  // namespace aruwlib
