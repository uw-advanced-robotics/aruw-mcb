#include "uart_1_dma.hpp"

#include "aruwlib/architecture/clock.hpp"
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

int iii = 0;
int jjj = 0;
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

    iii = modm::Clock::now().time_since_epoch().count() - jjj;
    jjj = modm::Clock::now().time_since_epoch().count();
}

using namespace aruwlib::arch;

using UartDma = Usart1Dma<
    DmaBase::ChannelSelection::CHANNEL_4,
    DmaBase::ChannelSelection::CHANNEL_4,
    Dma2::Stream2,
    Dma2::Stream7>;

#define UART_IT_MASK 0x0000FFFFU

#define __HAL_UART_GET_IT_SOURCE(__HANDLE__, __IT__)                                           \
    (((((__IT__) >> 28U) == 1U)                                                                \
          ? (__HANDLE__)->CR1                                                                  \
          : (((((uint32_t)(__IT__)) >> 28U) == 2U) ? (__HANDLE__)->CR2 : (__HANDLE__)->CR3)) & \
     (((uint32_t)(__IT__)) & UART_IT_MASK))

#define __HAL_UART_GET_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->SR & (__FLAG__)) == (__FLAG__))

#define UART_CR1_REG_INDEX 1U

#define UART_IT_IDLE ((uint32_t)(UART_CR1_REG_INDEX << 28U | USART_CR1_IDLEIE))

MODM_ISR(USART1)
{
    // Idle flag set
    // if ((USART1->SR & USART_SR_IDLE) && (USART1->SR & (1 << 28) ))
    if (__HAL_UART_GET_FLAG(USART1, USART_SR_IDLE) &&
        __HAL_UART_GET_IT_SOURCE(USART1, UART_IT_IDLE))
    {
        modm::platform::Usart1::clearIdleFlag();
        Dma2::Stream2::disable();
        if ((50 - DMA2_Stream2->NDTR) == 18)
        {
            rc_callback_handler();
        }
        Dma2::Stream2::setDataLength(50);
        Dma2::Stream2::enable();
    }
}
