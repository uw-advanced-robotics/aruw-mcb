#include <rm-dev-board-a/board.hpp>
#include "communication/cv_comms.hpp"
#include "communication/referee_comm.hpp"
uint8_t buff[1024] = {"a"};
uint16_t size = 0;
IMUData_t imu = {0};
ChassisData_t chassis = {0};
TurretAimData_t turret = {0};

TurretAimData_t turretOut = {0};

uint16_t interval = 3000;

ref_game_data_t game;
ref_robot_data_t robot;
static RefereeSystem ref;
int count = 0;



void boom(TurretAimData_t* d){
    count = count + 1;

}

int main()
{
    turret.yaw = 0;
    turret.pitch = 0;
    Board::initialize();
    Board::Leds::toggle();
    CVCommunication::initialize(3,boom);
    CVCommunication::beginTargetTracking();
    ref.initialize();
    ref.update(false,BASE_CTRL_MODE ,false, false);
    game = ref.getGameData();
    robot = ref.getRobotData();
    // CVCommunication::initialize(1);
    while (1)
    {


        CVCommunication::update(&imu, &chassis, &turret, robot.robot_id);
        CVCommunication::getLastAimData(&turretOut);
        modm::delayMicroseconds(interval);
        
    }
    return 0;
}
