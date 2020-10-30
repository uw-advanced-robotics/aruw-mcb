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
    static void initialize(uint8_t *buff)
    {
        // DmaStreamRx::configure();
        // DmaStreamRx::template setPeripheralRequest<Dma::RxRequest>();

        // TxChannel::configure(DmaBase::DataTransferDirection::MemoryToPeripheral,
        //         DmaBase::MemoryDataSize::Byte, DmaBase::PeripheralDataSize::Byte,
        //         DmaBase::MemoryIncrementMode::Increment, DmaBase::PeripheralIncrementMode::Fixed,
        //         DmaBase::Priority::High);
        // DmaStreamTx::setPeripheralAddress();
        // DmaStreamTx::setTransferErrorIrqHandler(handleDmaTransferError);
        // DmaStreamTx::setTransferCompleteIrqHandler(handleDmaTransmitComplete);
        // DmaStreamTx::enableInterruptVector();
        // DmaStreamTx::enableInterrupt(
        //     DmaBase::Interrupt::ERROR | DmaBase::Interrupt::TRANSFER_COMPLETE);
        // DmaStreamTx::template setPeripheralRequest<TxRequest>();

        modm::platform::Usart1::initialize<SystemClock, baudrate, tolerance>(12, modm::platform::UartBase::Parity::Even);

        DmaStreamRx::configure(ChannelIdRx,
            DmaBase::DataTransferDirection::PERIPHERAL_TO_MEMORY,
            DmaBase::MemoryDataSize::BYTE,
            DmaBase::PeripheralDataSize::BYTE,
            DmaBase::MemoryIncrementMode::INCREMENT,
            DmaBase::PeripheralIncrementMode::INCREMENT,
            DmaBase::CircularMode::DISABLED);

        USART1->CR3 |= USART_CR3_DMAR;

        // Write the USART_DR register address in the DMA control register to configure it as the
        // source of the transfer. The data will be moved from this address to the memory after
        // each RXNE event.
        DmaStreamRx::setSourceAddress((uintptr_t)&USART1->DR);

        // Write the memory address in the DMA control register to configure it as the destination
        // of the transfer. The data will be loaded from USART_DR to this memory area after each
        // RXNE event.
        DmaStreamRx::setDestinationAddress((uintptr_t)buff);

        // configure total # of bytes
        DmaStreamRx::setDataLength(50);

        // configure channel priority done above

        // configure interrupt generation
        DmaStreamRx::setTransferErrorIrqHandler(handleDmaTransferError);
        DmaStreamRx::setTransferCompleteIrqHandler(handleDmaReceiveComplete);
        DmaStreamRx::enableInterruptVector();
        DmaStreamRx::enableInterrupt(
            DmaBase::Interrupt::ERROR | DmaBase::Interrupt::TRANSFER_COMPLETE);

        // activate channel in DMA control register
        DmaStreamRx::enable();

        // When number of data transfers programmed in the DMA controller is reached, the DMA
        // controller generates an interrupt on the DMA channel interrupt vector The DMAR bit should
        // be cleared by software in the USART_Cr3 register during the interrupt subroutine

        // page 1003
    }

    static void writeBlocking(uint8_t data) {}

    static void writeBlocking(const uint8_t *data, std::size_t length) {}

    static void flushWriteBuffer() {}

    static bool write(uint8_t) { return false; }

    static std::size_t write(const uint8_t *, std::size_t) { return 0; }

    static bool isWriteFinished() { return false; }

    static std::size_t transmitBufferSize() { return 0; }

    static std::size_t discardTransmitBuffer() { return 0; }

    static bool read(uint8_t &) { return false; }

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

    static void handleDmaTransferError() { dmaError = true; }

    static void handleDmaReceiveComplete()
    {
        dmaComplete = true;
        DmaStreamRx::disable();
        uart1DmaMessageCompleteCallback();
    }

    static void handleDmaTransmitComplete() { DmaStreamTx::disable(); }

private:
    static inline bool dmaError = false;
    static inline bool dmaComplete = false;
};  // class Usart1Dma
}  // namespace arch
}  // namespace aruwlib

#endif  // UART_1_DMA_HPP_
