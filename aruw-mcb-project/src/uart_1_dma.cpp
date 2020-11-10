#include "uart_1_dma.hpp"

MODM_ISR(USART1) { aruwlib::arch::uart1IrqHandler(); }
