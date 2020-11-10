#include "uart_1_dma.hpp"

#include "aruwlib/architecture/clock.hpp"

MODM_ISR(USART1) { aruwlib::arch::uart1TxIrqHandler(); }
