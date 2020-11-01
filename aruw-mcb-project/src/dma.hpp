#ifndef DMA_HPP_
#define DMA_HPP_

#include <stdint.h>

#include <modm/platform/clock/rcc.hpp>

#include "dma_hal.hpp"

namespace aruwlib
{
namespace arch
{
/**
 * DMA controller
 */
template <uint32_t ID>
class DmaController : public DmaBase
{
    static_assert(ID == 1 || ID == 2, "dma id invalid");

public:
    /**
     * Enable the DMA controller in the RCC
     */
    static void enableRcc()
    {
        if constexpr (ID == 1)
        {
            modm::platform::Rcc::enable<modm::platform::Peripheral::Dma1>();
        }
        else
        {
            modm::platform::Rcc::enable<modm::platform::Peripheral::Dma2>();
        }
    }

    /**
     * Disable the DMA controller in the RCC
     */
    static void disableRcc()
    {
        if constexpr (ID == 1)
        {
            modm::platform::Rcc::disable<modm::platform::Peripheral::Dma1>();
        }
        else
        {
            modm::platform::Rcc::disable<modm::platform::Peripheral::Dma2>();
        }
    }

    /**
     * Class representing a DMA channel/stream
     */
    template <DmaBase::Stream StreamID>
    class Stream
    {
        using ControlHal = DmaHal<ID>;
        using StreamHal = DmaStreamHal<StreamID, ControlHal::StreamBase()>;

    public:
        static constexpr uint32_t DMA_ID = ID;
        static constexpr DmaBase::Stream STREAM_ID = StreamID;

        /**
         * Configure the DMA channel
         *
         * @see DmaStreamHal::configure
         */
        static void configure(
            ChannelSelection channel,
            DataTransferDirection direction,
            PeripheralIncrementMode periphInc,
            MemoryIncrementMode memInc,
            PeripheralDataSize peripDataSize,
            MemoryDataSize memDataSize,
            ControlMode mode,
            PriorityLevel priority,
            FifoMode fifoMode,
            FifoThreshold fifoThreshold,
            MemoryBurstTransfer memBurst,
            PeripheralBurstTransfer periphBurst)
        {
            StreamHal::configure(
                channel,
                direction,
                periphInc,
                memInc,
                peripDataSize,
                memDataSize,
                mode,
                priority,
                fifoMode,
                fifoThreshold,
                memBurst,
                periphBurst);
        }

        static void selectChannel(DmaBase::ChannelSelection channel)
        {
            StreamHal::selectChannel(channel);
        }

        /**
         * Start the transfer of the DMA channel
         */
        static void enable()
        {
            ControlHal::clearInterruptFlags(StreamID);
            StreamHal::enable();
        }
        /**
         * Stop a DMA channel transfer
         */
        static void disable() { StreamHal::disable(); }

        /**
         * Get the direction of the data transfer
         */
        static DataTransferDirection getDataTransferDirection()
        {
            return StreamHal::getDataTransferDirection();
        }

        /**
         * Set the memory address of the DMA channel
         *
         * @note In Mem2Mem mode use this method to set the memory source address.
         *
         * @param[in] address Source address
         */
        static void setSourceAddress(uintptr_t address) { StreamHal::setSourceAddress(address); }

        /**
         * Set the peripheral address of the DMA channel
         *
         * @note In Mem2Mem mode use this method to set the memory destination address.
         *
         * @param[in] address Destination address
         */
        static void setDestinationAddress(uintptr_t address)
        {
            StreamHal::setDestinationAddress(address);
        }

        /**
         * Enable/disable memory increment
         *
         * When enabled, the memory address is incremented by the size of the data
         * (e.g. 1 for byte transfers, 4 for word transfers) after the transfer
         * completed.
         *
         * @param[in] increment Enable/disable
         */
        static void setMemoryIncrementMode(DmaBase::MemoryIncrementMode mode)
        {
            StreamHal::setMemoryIncrementMode(mode);
        }

        /**
         * Enable/disable peripheral increment
         *
         * When enabled, the peripheral address is incremented by the size of the data
         * (e.g. 1 for byte transfers, 4 for word transfers) after the transfer
         * completed.
         *
         * @param[in] increment Enable/disable
         */
        static void setPeripheralIncrementMode(DmaBase::PeripheralIncrementMode mode)
        {
            StreamHal::setPeripheralIncrementMode(mode);
        }

        /**
         * Set the length of data to be transfered
         */
        static void setDataLength(std::size_t length) { StreamHal::setDataLength(length); }

        /**
         * Set the IRQ handler for transfer errors
         *
         * The handler will be called from the channels IRQ handler function
         * when the IRQ status indicates an error occured.
         */
        static void setTransferErrorIrqHandler(IrqHandler irqHandler)
        {
            transferError = irqHandler;
        }
        /**
         * Set the IRQ handler for transfer complete
         *
         * Called by the channels IRQ handler when the transfer is complete.
         */
        static void setTransferCompleteIrqHandler(IrqHandler irqHandler)
        {
            transferComplete = irqHandler;
        }
        /**
         * Set the peripheral that operates the channel
         */
        // template <DmaBase::Request dmaRequest>
        // static void
        // setPeripheralRequest()
        // {
        //     // DMA_Request_TypeDef *DMA_REQ = reinterpret_cast<DMA_Request_TypeDef
        //     *>(ControlHal::DMA_CSEL);
        //     // DMA_REQ->CSELR &= ~(0x0f << (uint32_t(ChannelID) * 4));
        //     // DMA_REQ->CSELR |= uint32_t(dmaRequest) << (uint32_t(ChannelID) * 4);
        // }

        /**
         * IRQ handler of the DMA channel
         *
         * Reads the IRQ status and checks for error or transfer complete. In case
         * of error the DMA channel will be disabled.
         */
        static void interruptHandler()
        {
            uint32_t currIsrs = ControlHal::getInterruptFlags(StreamID);
            ControlHal::clearInterruptFlags(StreamID);
            if (currIsrs & (uint32_t(InterruptStatusMsks::TRANSFER_ERROR) |
                            uint32_t(InterruptStatusMsks::DIRECT_MODE_ERROR) |
                            uint32_t(InterruptStatusMsks::FIFO_ERROR)))
            {
                if (transferError != nullptr)
                {
                    transferError();
                }
            }
            if ((currIsrs & uint32_t(InterruptStatusMsks::TRANSFER_COMPLETE)) && transferComplete)
            {
                transferComplete();
            }
        }

        /**
         * Enable the IRQ vector of the channel
         *
         * @param[in] priority Priority of the IRQ
         */
        static void enableInterruptVector(uint32_t priority = 1)
        {
            NVIC_SetPriority(DmaBase::Nvic<ID>::DmaIrqs[uint32_t(StreamID)], priority);
            NVIC_EnableIRQ(DmaBase::Nvic<ID>::DmaIrqs[uint32_t(StreamID)]);
        }

        /**
         * Disable the IRQ vector of the channel
         */
        static void disableInterruptVector()
        {
            NVIC_DisableIRQ(DmaBase::Nvic<ID>::DmaIrqs[uint32_t(StreamID)]);
        }

        /**
         * Enable the specified interrupt of the channel
         */
        static void enableInterrupt(Interrupt_t irq) { StreamHal::enableInterrupt(irq); }
        /**
         * Disable the specified interrupt of the channel
         */
        static void disableInterrupt(Interrupt_t irq)
        {
            // StreamHal::disableInterrupt(irq);
        }

    private:
        static inline DmaBase::IrqHandler transferError{nullptr};
        static inline DmaBase::IrqHandler transferComplete{nullptr};
    };
};  // class DmaController

/*
 * Derive DMA controller classes for convenience. Every derived class defines
 * the channels available on that controller.
 */
class Dma1 : public DmaController<1>
{
public:
    using Stream0 = DmaController<1>::Stream<DmaBase::Stream::STREAM_0>;
    using Stream1 = DmaController<1>::Stream<DmaBase::Stream::STREAM_1>;
    using Stream2 = DmaController<1>::Stream<DmaBase::Stream::STREAM_2>;
    using Stream3 = DmaController<1>::Stream<DmaBase::Stream::STREAM_3>;
    using Stream4 = DmaController<1>::Stream<DmaBase::Stream::STREAM_4>;
    using Stream5 = DmaController<1>::Stream<DmaBase::Stream::STREAM_5>;
    using Stream6 = DmaController<1>::Stream<DmaBase::Stream::STREAM_6>;
    using Stream7 = DmaController<1>::Stream<DmaBase::Stream::STREAM_7>;
};  // class Dma1

class Dma2 : public DmaController<2>
{
public:
    using Stream0 = DmaController<2>::Stream<DmaBase::Stream::STREAM_0>;
    using Stream1 = DmaController<2>::Stream<DmaBase::Stream::STREAM_1>;
    using Stream2 = DmaController<2>::Stream<DmaBase::Stream::STREAM_2>;
    using Stream3 = DmaController<2>::Stream<DmaBase::Stream::STREAM_3>;
    using Stream4 = DmaController<2>::Stream<DmaBase::Stream::STREAM_4>;
    using Stream5 = DmaController<2>::Stream<DmaBase::Stream::STREAM_5>;
    using Stream6 = DmaController<2>::Stream<DmaBase::Stream::STREAM_6>;
    using Stream7 = DmaController<2>::Stream<DmaBase::Stream::STREAM_7>;
};  // class Dma2
}  // namespace arch
}  // namespace aruwlib

#endif  // DMA_HPP_
