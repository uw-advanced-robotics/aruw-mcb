#ifndef UART_1_DMA_HPP_
#define UART_1_DMA_HPP_

#include <modm/platform/uart/uart_1.hpp>

#include "dma.hpp"
#include "dma_request_mapping.hpp"

namespace aruwlib
{
namespace arch
{
template <
    DmaBase::ChannelSelection ChannelIdRx,
    DmaBase::ChannelSelection ChannelIdTx,
    typename DmaStreamRx,
    typename DmaStreamTx>
class Usart1Dma : modm::platform::Usart1
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
    static void initialize()
    {
        // DmaStreamRx::configure();
        DmaStreamRx::setTransferErrorIrqHandler(handleDmaTransferError);
        DmaStreamRx::setTransferCompleteIrqHandler(handleDmaReceiveComplete);
        DmaStreamRx::enableInterruptVector();
        DmaStreamRx::enableInterrupt(
            DmaBase::Interrupt::ERROR | DmaBase::Interrupt::TRANSFER_COMPLETE);
        // DmaStreamRx::template setPeripheralRequest<Dma::RxRequest>();

        // TxChannel::configure(DmaBase::DataTransferDirection::MemoryToPeripheral,
        //         DmaBase::MemoryDataSize::Byte, DmaBase::PeripheralDataSize::Byte,
        //         DmaBase::MemoryIncrementMode::Increment, DmaBase::PeripheralIncrementMode::Fixed,
        //         DmaBase::Priority::High);
        // DmaStreamTx::setPeripheralAddress();
        DmaStreamTx::setTransferErrorIrqHandler(handleDmaTransferError);
        DmaStreamTx::setTransferCompleteIrqHandler(handleDmaTransmitComplete);
        DmaStreamTx::enableInterruptVector();
        DmaStreamTx::enableInterrupt(
            DmaBase::Interrupt::ERROR | DmaBase::Interrupt::TRANSFER_COMPLETE);
        // DmaStreamTx::template setPeripheralRequest<TxRequest>();

        modm::platform::Usart1::initialize<SystemClock, baudrate, tolerance>();
    }

    static void writeBlocking(uint8_t data) {}

    static void writeBlocking(const uint8_t *data, std::size_t length) {}

    static void flushWriteBuffer() {}

    static bool write(uint8_t data) {}

    static std::size_t write(const uint8_t *data, std::size_t length) {}

    static bool isWriteFinished() {}

    static std::size_t transmitBufferSize() {}

    static std::size_t discardTransmitBuffer() {}

    static bool read(uint8_t &data) {}

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

    static std::size_t receiveBufferSize() {}

    static std::size_t discardReceiveBuffer() {}

    static bool hasError() {}

    static void clearError() {}

    static void handleDmaTransferError() {}

    static void handleDmaReceiveComplete() { DmaStreamRx::disable(); }

    static void handleDmaTransmitComplete() { DmaStreamTx::disable(); }

private:
    static inline bool dmaError = false;
};  // class Usart1Dma
}  // namespace arch
}  // namespace aruwlib

#endif  // UART_1_DMA_HPP_
