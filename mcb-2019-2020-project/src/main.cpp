#include <rm-dev-board-a/board.hpp>
int size=0;
uint8_t buff[1024];

int main()
{
    Board::initialize();
    Usart6::connect<GpioG14::Tx,GpioG9::Rx>();
    Usart6::initialize<Board::SystemClock,115200>();
    while (1)
    {
        size=Usart6::RxBufferSize;
        modm::delayMilliseconds(1);
        if (Usart6::RxBufferSize)
        {
            Usart6::read(buff,size);
            Usart6::writeBlocking(buff,size);
        }
        
    }
    return 0;
}
