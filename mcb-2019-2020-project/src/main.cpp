#include <rm-dev-board-a/board.hpp>
#include "src/motor/dji_motor_tx_handler.hpp"
#include "src/communication/can/can_rx_handler.hpp"
#include "src/communication/can/can_bluepill.hpp"
#include <modm/platform/can/can_1.hpp>

uint32_t i = 0;

aruwlib::can::CanBluepill testBluePill(0x201);

int main()
{
    Board::initialize();

    while (1)
    {
        i++;
        if (i > 1000) {
            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
            Board::Leds::toggle();
            i = 0; 
        }
        aruwlib::can::CanRxHandler::pollCanData();
        modm::delayMicroseconds(10);
        modm::can::Message newMessage;

        newMessage.identifier = 0x201; 
        newMessage.setExtended(false);

        testBluePill.transferMessage(newMessage, 8, 1, 34, 45, 86, true); 
    }
    return 0;
}
