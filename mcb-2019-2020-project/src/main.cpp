#include <rm-dev-board-a/board.hpp>
#include "communication/cv_comms.hpp"
#include "communication/referee_comm.hpp"
uint8_t buff[1024] = {"a"};
uint16_t size = 0;
IMUData_t imu = {0};
ChassisData_t chassis = {0};
uint16_t interval = 3000;
ref_game_data_t game;
ref_robot_data_t robot;
static RefereeSystem ref;
int main()
{
    Board::initialize();
    Board::Leds::toggle();
    ref.initialize();
    // CVCommunication::initialize(1);
    while (1)
    {
        // CVCommunication::update(&imu, &chassis, 1);
        game = ref.getGameData();
        robot = ref.getRobotData();
        ref.update(false,BASE_CTRL_MODE ,false, false);
        modm::delayMicroseconds(interval);
        
    }
    return 0;
}
