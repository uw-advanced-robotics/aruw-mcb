#include <rm-dev-board-a/board.hpp>
#include "src/aruwlib/communication/serial/ref_comm.hpp"
#include "src/aruwlib/communication/serial/dji_serial.hpp"

#include <modm/platform/uart/uart_2.hpp>

using namespace aruwlib::serial;

RefereeSystem::Game_Data_t game;
RefereeSystem::Robot_Data_t robot;

int main()
{
    Board::initialize();
    RefereeSystem::initialize();
    while (1)
    {
        if (Usart2::isWriteFinished()) {
            Usart2::write(50);
        }
        game = RefereeSystem::getGameData();
        robot = RefereeSystem::getRobotData();
        RefereeSystem::periodicTask();
        modm::delayMicroseconds(10);
    }
    return 0;
}