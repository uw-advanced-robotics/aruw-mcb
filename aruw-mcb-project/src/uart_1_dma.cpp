#include "uart_1_dma.hpp"


extern uint8_t dmatxbuffer[50];
typedef struct
{
  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;

  /* left and right lever information */
  uint8_t sw1;
  uint8_t sw2;
} rc_info_t;

rc_info_t rc;

void rc_callback_handler()
{
    uint8_t *buff = dmatxbuffer;
  rc.ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
  rc.ch1 -= 1024;
  rc.ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
  rc.ch2 -= 1024;
  rc.ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
  rc.ch3 -= 1024;
  rc.ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
  rc.ch4 -= 1024;

  rc.sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
  rc.sw2 = (buff[5] >> 4) & 0x0003;
}

int iii=0;
int jjj=0;

using namespace aruwlib::arch;

using UartDma = Usart1Dma<
    DmaBase::ChannelSelection::CHANNEL_4,
    DmaBase::ChannelSelection::CHANNEL_4,
    Dma2::Stream2,
    Dma2::Stream7>;

MODM_ISR(USART1)
{
    // Idle flag set
    if (USART1->SR & USART_SR_IDLE)
    {
        modm::platform::Usart1::clearIdleFlag();
        jjj++;
        iii=DMA2_Stream2->NDTR;
        Dma2::Stream2::disable();
        if ((48 - DMA2_Stream2->NDTR) == 18)
        {
            rc_callback_handler();
        }
        Dma2::Stream2::setDataLength(48);
        Dma2::Stream2::enable();
    }
}
