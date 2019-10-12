#include <rm-dev-board-a/board.hpp>
uint8_t buff[1024]={"a"};
uint16_t size=0;

int main()
{
    Board::initialize();
    Usart6::connect<GpioG14::Tx,GpioG9::Rx>();
    Usart6::initialize<Board::SystemClock,115200>();
    while (1)
    {
        size=Usart6::getRxBufferSize();
        if(size){
            Usart6::write(buff,1);
            Usart6::read(&buff[2],1);
            
        }
        modm::delayMilliseconds(1);

        
    }
    return 0;
}
