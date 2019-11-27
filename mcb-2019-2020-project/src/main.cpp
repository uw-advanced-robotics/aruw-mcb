#include <rm-dev-board-a/board.hpp>
#include "communication/referee_comm.hpp"
#include "communication/cv_comm.hpp"
#include "communication/serial.hpp"
using namespace aruwlib;
using namespace serial;
RefereeSystem::Game_Data_t game;
RefereeSystem::Robot_Data_t robot;
CVCommunication::CV_IMU_Data_t imu;
CVCommunication::CV_Turret_Aim_Data_t turrent;
CVCommunication::CV_Chassis_Data_t chassis;
void boom(CVCommunication::CV_Turret_Aim_Data_t* aaa) {}

int main()
{
    Board::initialize();
    RefereeSystem::initialize();
    CVCommunication::initialize(DJISerial::PORT_UART2, boom);
    while (1)
    {
        // Board::Leds::toggle();
        CVCommunication::periodicTask(&imu, &chassis, &turrent);
        game = RefereeSystem::getGameData();
        robot = RefereeSystem::getRobotData();
        RefereeSystem::periodicTask();
        modm::delayMilliseconds(5);
    }
    return 0;
}
 