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

template <
    DmaBase::ChannelSelection ChannelIdRx,
    DmaBase::ChannelSelection ChannelIdTx,
    typename DmaStreamRx,
    typename DmaStreamTx>
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

    template <
        typename SystemClock,
        modm::baudrate_t baudrate,
        modm::percent_t tolerance = modm::pct(1)>
    static void initialize(modm::platform::UartBase::Parity parity, std::size_t length, uint8_t *data)
    {
        modm::platform::Usart1::initialize<SystemClock, baudrate, tolerance>(12, parity);
        USART1->BRR = 0x384;
        USART1->CR1 &= ~USART_CR1_RXNEIE;

        // // Configure tx
        // USART1->CR3 |= USART_CR3_DMAT;

        // // Disable to allow modification of the DMA configuration register.
        // DmaStreamTx::disable();

        // DmaStreamTx::configure(
        //     ChannelIdTx,
        //     DmaBase::DataTransferDirection::MEMORY_TO_PERIPHERAL,
        //     DmaBase::MemoryDataSize::BYTE,
        //     DmaBase::PeripheralDataSize::BYTE,
        //     DmaBase::MemoryIncrementMode::INCREMENT,
        //     DmaBase::PeripheralIncrementMode::INCREMENT,
        //     DmaBase::CircularMode::DISABLED);

        // // Write the memory address in the DMA configuration register to configure it as the source
        // // of the transfer. The data will be loaded into the USART_DR register from this memory area
        // // after each TXE event.
        // DmaStreamTx::setSourceAddress((uintptr_t)&USART1->DR);

        // // Clear the TC bit in the SR register by writing 0 to it.
        // USART1->SR &= ~USART_SR_TC;

        // // Configure destination address, size of bytes to write in write functions

        // // Configure DMA interrupt generation
        // DmaStreamTx::setTransferErrorIrqHandler(handleDmaTransferError);
        // DmaStreamTx::setTransferCompleteIrqHandler(handleDmaTransmitComplete);
        // DmaStreamTx::enableInterruptVector();
        // DmaStreamTx::enableInterrupt(
        //     DmaBase::Interrupt::ERROR | DmaBase::Interrupt::TRANSFER_COMPLETE);

        // See page 1003 in STM32F405/415, STM32F407/417, STM32F427/437 and STM32F429/439 advanced
        // Arm®-based 32-bit MCUs - Reference manual

        DmaStreamRx::configure(
            ChannelIdRx,
            DmaBase::DataTransferDirection::PERIPHERAL_TO_MEMORY,
            DmaBase::PeripheralIncrementMode::FIXED,
            DmaBase::MemoryIncrementMode::INCREMENT,
            DmaBase::PeripheralDataSize::BYTE,
            DmaBase::MemoryDataSize::BYTE,
            DmaBase::ControlMode::CIRCULAR,
            DmaBase::PriorityLevel::LOW,
            DmaBase::FifoMode::DISABLED,
            DmaBase::FifoThreshold::QUARTER_FULL,
            DmaBase::MemoryBurstTransfer::SINGLE,
            DmaBase::PeripheralBurstTransfer::SINGLE);

        // Write the USART_DR register address in the DMA control register to configure it as the
        // source of the transfer. The data will be moved from this address to the memory after
        // each RXNE event.
        DmaStreamRx::setSourceAddress((uintptr_t)&USART1->DR);

        // configure interrupt generation
        DmaStreamRx::setTransferErrorIrqHandler(handleDmaReceiveError);
        DmaStreamRx::setTransferCompleteIrqHandler(handleDmaReceiveComplete);
        DmaStreamRx::enableInterruptVector();
        // DmaStreamRx::enableInterrupt(DmaBase::Interrupt::TRANSFER_COMPLETE);
        // DmaStreamRx::enableInterrupt(DmaBase::Interrupt::ERROR);
        modm::platform::UsartHal1::enableInterrupt(modm::platform::UartBase::Interrupt::RxIdle);

        clearIdleFlag();

        // Start RX, probably move elsewhere
        DmaStreamRx::setDataLength(length);
        DmaStreamRx::setDestinationAddress((uintptr_t) data);
        DmaStreamRx::enable();
        USART1->CR3 |= USART_CR3_DMAR;
    }

    /**
     * Configure USART1 to write to the requested data buffer length bytes. When the DMA message
     * received interrupted is triggered, the messageReceivedHandler will be called.
     *
     * @note When in continuous read mode, other read operations are disabled.
     */
    static void configureContinuousRead(uint8_t *data, std::size_t length, IrqHandler)
    {
        rxLength = length;
        // Disable to allow modifications of the DMA stream control register.
        DmaStreamRx::disable();
        // configure total # of bytes
        DmaStreamRx::setDataLength(length);
        // Write the memory address in the DMA control register to configure it as the destination
        // of the transfer. The data will be loaded from USART_DR to this memory area after each
        // RXNE event.
        DmaStreamRx::setDestinationAddress((uintptr_t)data);
        // activate channel in DMA control register
        DmaStreamRx::enable();
    }

    // static void writeBlocking(uint8_t data) {}

    // static void writeBlocking(const uint8_t *data, std::size_t length) {}

    // static void flushWriteBuffer() {}

    // static bool write(uint8_t) { return false; }

    // static std::size_t write(const uint8_t *, std::size_t) { return 0; }

    // static bool isWriteFinished() { return false; }

    // static std::size_t transmitBufferSize() { return 0; }

    // static std::size_t discardTransmitBuffer() { return 0; }

    // static bool read(uint8_t &) { return false; }

    static std::size_t read(uint8_t *buffer, std::size_t length)
    {
        if (buffer == nullptr || length < 0)
        {
            return 0;
        }
        DmaStreamRx::setSourceAddress(buffer);
        DmaStreamRx::setDataLength(length);
        DmaStreamRx::enable();
    }

    static std::size_t receiveBufferSize() { return 0; }

    static std::size_t discardReceiveBuffer() { return 0; }

    static bool hasError() { return dmaError; }

    static void clearError() {}

    static void handleDmaReceiveError() {}

    static void handleDmaTransferError() { dmaError = true; }

    static void handleDmaReceiveComplete()
    {
        // When number of data transfers programmed in the DMA controller is reached, the DMA
        // controller generates an interrupt on the DMA channel interrupt vector The DMAR bit should
        // be cleared by software in the USART_Cr3 register during the interrupt subroutine
        DmaStreamRx::disable();
        uart1DmaMessageCompleteCallback();
        if (rxLength != 0)
        {
            // Restart stream if in contiguous receive mode
            DmaStreamRx::setDataLength(rxLength);
            DmaStreamRx::enable();
        }
        else
        {
            USART1->CR3 &= ~USART_CR3_DMAR;
        }
    }

    static void handleDmaTransmitComplete() { DmaStreamTx::disable(); }

private:
    static inline bool dmaError = false;
    static std::size_t rxLength;
};  // class Usart1Dma

template <
    DmaBase::ChannelSelection ChannelIdRx,
    DmaBase::ChannelSelection ChannelIdTx,
    typename DmaStreamRx,
    typename DmaStreamTx>
std::size_t Usart1Dma<ChannelIdRx, ChannelIdTx, DmaStreamRx, DmaStreamTx>::rxLength = 0;
}  // namespace arch
}  // namespace aruwlib

#endif  // UART_1_DMA_HPP_
