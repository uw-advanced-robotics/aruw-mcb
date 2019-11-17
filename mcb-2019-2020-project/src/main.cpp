#include <rm-dev-board-a/board.hpp>
#include "communication/referee_comm.hpp"
#include "communication/cv_comm.hpp"
#include "communication/serial.hpp"
using namespace aruwlib;
using namespace serial;
ref_game_data_t game;
ref_robot_data_t robot;
void boom(CVCommunication::CV_Turret_Aim_Data_t* aaa){

}
int main()
{
    Board::initialize();
    RefereeSystem::initialize();
    while (1)
    {
        //Board::Leds::toggle();
        game = RefereeSystem::getGameData();
        robot = RefereeSystem::getRobotData();
        RefereeSystem::periodicTask();
        modm::delayMilliseconds(5);
    }
    return 0;
}
 