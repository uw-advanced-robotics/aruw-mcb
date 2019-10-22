#include <rm-dev-board-a/board.hpp>
#include "communication/cv_comms.hpp"

uint8_t buff[1024]={"a"};
uint16_t size=0;
CVCommunication::IMUData_t imu = {0};
CVCommunication::ChassisData_t chassis = {0};
uint16_t interval = 5; //3ms is the minimum time for a successfully imu data sending
int main()
{
    Board::initialize();
    Board::Leds::toggle();
    CVCommunication::initialize(1);
    while (1)
    {

        //CVCommunication::update(&imu, &chassis, 1);
        modm::delayMilliseconds(interval);

        
    }
    return 0;
}
