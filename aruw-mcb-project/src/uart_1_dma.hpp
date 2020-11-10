#ifndef UART_1_DMA_HPP_
#define UART_1_DMA_HPP_

#include <modm/platform/uart/uart_1.hpp>

#include "dma.hpp"
#include "dma_request_mapping.hpp"

// __weak void uart1DmaMessageReceived(void);

namespace aruwlib
{
namespace arch
{
__attribute__((weak)) void uart1DmaMessageCompleteCallback(void);
__attribute__((weak)) void uart1TxIrqHandler(void);

template <
    typename DmaStreamTx,
    DmaBase::ChannelSelection ChannelIdRx,
    typename DmaStreamRx,
    DmaBase::ChannelSelection ChannelIdTx>
class Usart1Dma : public modm::platform::Usart1
{
public:
    using IrqHandler = void (*)();

    static_assert(
        DmaRequestMapping::validateRequestMapping<
            DmaStreamRx::DMA_ID,
            ChannelIdRx,
            DmaStreamRx::STREAM_ID,
            DmaRequestMapping::Peripheral::USART1_RX>(),
        "dma usart1 tx channel or stream selection invalid");

    static_assert(
        DmaRequestMapping::validateRequestMapping<
            DmaStreamTx::DMA_ID,
            ChannelIdTx,
            DmaStreamTx::STREAM_ID,
            DmaRequestMapping::Peripheral::USART1_TX>(),
        "dma usart1 tx channel or stream selection invalid");

    /**
     * Performs DMA/UART specific integration configuration.
     *
     * @note DmaStreamRx and DmaStreamTx should be pre-configured (configuration is not done in
     *      this function). Also, this function **does not** configure any interrupts. The user
     *      may choose to do this.
     */
    template <
        typename SystemClock,
        modm::baudrate_t baudrate,
        modm::percent_t tolerance = modm::pct(1)>
    static void initialize(modm::platform::UartBase::Parity parity)
    {
        modm::platform::Usart1::initialize<SystemClock, baudrate, tolerance>(12, parity);

        // See page 1003 in STM32F405/415, STM32F407/417, STM32F427/437 and STM32F429/439 advanced
        // Arm-based 32-bit MCUs - Reference manual

        // TX

        DmaStreamTx::disable();

        // Write the memory address in the DMA configuration register to configure it as the
        // source of the transfer. The data will be loaded into the USART_DR register from this
        // memory area after each TXE event.
        DmaStreamTx::setSourceAddress(reinterpret_cast<uintptr_t>(&USART1->DR));

        // Clear the TC bit in the SR register by writing 0 to it.
        USART1->SR &= ~USART_SR_TC;

        // Enable UART DMA TX
        USART1->CR3 |= USART_CR3_DMAT;

        // RX

        // Write the USART_DR register address in the DMA control register to configure it as the
        // source of the transfer. The data will be moved from this address to the memory after
        // each RXNE event.
        DmaStreamRx::setSourceAddress(reinterpret_cast<uintptr_t>(&USART1->DR));

        // Enable UART DMA RX
        USART1->CR3 |= USART_CR3_DMAR;
    }

    /**
     * Configure USART1 to write to the requested data buffer length bytes. When the DMA message
     * received interrupted is triggered, the messageReceivedHandler will be called.
     *
     * @note When in continuous read mode, other read operations are disabled.
     */
    static void configureContinuousRead(uint8_t *data, std::size_t length)
    {
        // Disable to allow modifications of the DMA stream control register.
        DmaStreamRx::disable();
        // configure total # of bytes
        DmaStreamRx::setDataLength(length);
        // Write the memory address in the DMA control register to configure it as the destination
        // of the transfer. The data will be loaded from USART_DR to this memory area after each
        // RXNE event.
        DmaStreamRx::setDestinationAddress(reinterpret_cast<uintptr_t>(data));
        // activate channel in DMA control register
        DmaStreamRx::enable();
    }

    static void handleDmaReceiveComplete()
    {
        // When number of data transfers programmed in the DMA controller is reached, the DMA
        // controller generates an interrupt on the DMA channel interrupt vector The DMAR bit should
        // be cleared by software in the USART_Cr3 register during the interrupt subroutine
        DmaStreamRx::disable();
        uart1DmaMessageCompleteCallback();
        USART1->CR3 &= ~USART_CR3_DMAR;
    }
};  // class Usart1Dma
}  // namespace arch
}  // namespace aruwlib

#endif  // UART_1_DMA_HPP_
