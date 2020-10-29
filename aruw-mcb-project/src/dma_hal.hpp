#ifndef DMA_HAL_HPP_
#define DMA_HAL_HPP_

#include <modm/architecture/interface/assert.hpp>

#include "dma_base.hpp"

namespace aruwlib
{
namespace arch
{
/**
 * Hardware abstraction of DMA controller
 */
template <uint32_t ID>
class DmaHal : public DmaBase
{
public:
    static_assert(ID == 1 || ID == 2, "invalid dma id");

    static constexpr uint32_t StreamBase()
    {
        if constexpr (ID == 1)
        {
            return DMA1_Stream0_BASE;
        }
        else
        {
            return DMA2_Stream0_BASE;
        }
    }

    static constexpr DMA_TypeDef *DmaDef()
    {
        if constexpr (ID == 1)
        {
            return DMA1;
        }
        else
        {
            return DMA2;
        }
    }

    /**
     * Sets the bits associated with the stream's interrupt flag clear register to high
     * (either LIFCR or HIFCR).
     */
    static void clearInterruptFlags(Stream s)
    {
        if (uint32_t(s) > uint32_t(Stream::STREAM_3))
        {
            DmaDef()->HIFCR |= dmaStreamIFCRMasks[uint32_t(s)];
        }
        else
        {
            DmaDef()->LIFCR |= dmaStreamIFCRMasks[uint32_t(s)];
        }
    }

private:
    static uint32_t getLowInterruptFlags() { return DmaDef()->LISR; }

    static uint32_t getHighInterruptFlags() { return DmaDef()->HISR; }

public:
    /**
     * Interrupt flag, use InterruptStatusMsks enum masks to interpret the interrupt flags
     */
    static uint32_t getInterruptFlags(Stream s)
    {
        if (uint32_t(s) > uint32_t(Stream::STREAM_3))
        {
            return (getHighInterruptFlags() >> (uint32_t(s) - uint32_t(Stream::STREAM_4))) &
                   INTERRUPT_STATUS_MSK;
        }
        else
        {
            return (getLowInterruptFlags() >> uint32_t(s)) & INTERRUPT_STATUS_MSK;
        }
    }
};  // class DmaHal

/**
 * ID must match the DMA number for the stream
 */
template <DmaBase::Stream S, uint32_t StreamBase>
class DmaStreamHAL : public DmaBase
{
public:
    static_assert(StreamBase == DMA1_Stream0_BASE || StreamBase == DMA2_Stream0_BASE, "StreamBase invalid");

#define STREAM_PTR(S, StreamBase) (((DMA_Stream_TypeDef *) StreamBase)[uint32_t(S)])

    /**
     * Configure the DMA stream (HAL)
     *
     * Stops the DMA stream and writes the new values to its control register.
     *
     * @param[in] direction Direction of the DMA stream
     * @param[in] memoryDataSize Size of data in memory (byte, halfword, word)
     * @param[in] peripheralDataSize Size of data in peripheral (byte, halfword, word)
     * @param[in] memoryIncrement Defines whether the memory address is incremented
     * 			  after a transfer completed
     * @param[in] peripheralIncrement Defines whether the peripheral address is
     * 			  incremented after a transfer completed
     * @param[in] priority Priority of the DMA stream
     * @param[in] circularMode Transfer data in circular mode?
     */
    static void configure(
        DataTransferDirection direction,
        MemoryDataSize memoryDataSize,
        PeripheralDataSize peripheralDataSize,
        MemoryIncrementMode memoryIncrement,
        PeripheralIncrementMode peripheralIncrement,
        CircularMode circularMode,
        PeripheralIncrementOffsetSize peripheralIncrementOffsetSize =
            PeripheralIncrementOffsetSize::LINKED_TO_PSIZE,
        PriorityLevel priority = PriorityLevel::MEDIUM,
        MemoryBurstTransfer memoryBurstMode = MemoryBurstTransfer::SINGLE,
        PeripheralBurstTransfer peripheralBurstMode = MemoryBurstTransfer::SINGLE)
    {
        disable();

        STREAM_PTR(S, StreamBase)->CR = uint32_t(direction) | uint32_t(memoryDataSize) | uint32_t(peripheralDataSize) |
                     uint32_t(memoryIncrement) | uint32_t(peripheralIncrement) |
                     uint32_t(peripheralIncrementOffsetSize) | uint32_t(priority) |
                     uint32_t(circularMode) | uint32_t(memoryBurstMode) |
                     uint32_t(peripheralBurstMode);
    }

    static void configureDoubleBufferMode(DoubleBufferMode bufferMode)
    {
        // TODO
        STREAM_PTR(S, StreamBase)->CR |= uint32_t(bufferMode);
    }

    static void enableInterrupts(Interrupt_t irq) { STREAM_PTR(S, StreamBase)->CR |= irq.value; }

    static void clearInterruptFlags() { STREAM_PTR(S, StreamBase)->FCR &= (DMA_SxFCR_FEIE); }

    /**
     * Enable the DMA stream to send/receive.
     */
    static void enable() { STREAM_PTR(S, StreamBase)->CR |= uint32_t(StreamEnableFlag::STREAM_ENABLED); }

    /**
     * Disable send/receive on the DMA stream
     */
    static void disable()
    {
        STREAM_PTR(S, StreamBase)->CR &= ~uint32_t(StreamEnableFlag::STREAM_ENABLED);
        // wait for stream to be stopped
        while (STREAM_PTR(S, StreamBase)->CR & uint32_t(StreamEnableFlag::STREAM_ENABLED))
            ;
    }

    static void selectChannel(ChannelSelection channel)
    {
        STREAM_PTR(S, StreamBase)->CR &= (~DMA_SxCR_CHSEL_Msk | uint32_t(channel));
    }

    /**
     * Get the direction of the data transfer
     */
    static DataTransferDirection getDataTransferDirection()
    {
        return static_cast<DataTransferDirection>(STREAM_PTR(S, StreamBase)->CR & (DMA_SxCR_DIR_0 | DMA_SxCR_DIR_1));
    }

    static void setSourceAddress(uintptr_t src)
    {
        if ((STREAM_PTR(S, StreamBase)->CR & uint32_t(DataTransferDirection::MEMORY_TO_MEMORY)) ==
            uint32_t(DataTransferDirection::MEMORY_TO_MEMORY))
        {
            STREAM_PTR(S, StreamBase)->PAR = src;
        }
        else if (
            (STREAM_PTR(S, StreamBase)->CR & uint32_t(DataTransferDirection::MEMORY_TO_PERIPHERAL)) ==
            uint32_t(DataTransferDirection::MEMORY_TO_PERIPHERAL))
        {
            STREAM_PTR(S, StreamBase)->M0AR = src;
        }
        else
        {
            STREAM_PTR(S, StreamBase)->PAR = src;
        }
    }

    static void setDestinationAddress(uintptr_t dst)
    {
        if ((STREAM_PTR(S, StreamBase)->CR & uint32_t(DataTransferDirection::MEMORY_TO_MEMORY)) ==
            uint32_t(DataTransferDirection::MEMORY_TO_MEMORY))
        {
            STREAM_PTR(S, StreamBase)->M0AR = dst;
        }
        else if (
            (STREAM_PTR(S, StreamBase)->CR & uint32_t(DataTransferDirection::MEMORY_TO_PERIPHERAL)) ==
            uint32_t(DataTransferDirection::MEMORY_TO_PERIPHERAL))
        {
            STREAM_PTR(S, StreamBase)->PAR = dst;
        }
        else
        {
            STREAM_PTR(S, StreamBase)->M0AR = dst;
        }
    }

    static void setDataLength(std::size_t length) { STREAM_PTR(S, StreamBase)->NDTR = length; }

    // /**
    //  * IRQ handler of the DMA channel
    //  *
    //  * Reads the IRQ status and checks for error or transfer complete. In case
    //  * of error the DMA channel will be disabled.
    //  */
    // static void interruptHandler()
    // {
    //     uint32_t currIsrs = DMA::getInterruptFlags();
    //     if (currIsrs & (InterruptStatusMsks::TRANSFER_ERROR |
    //                     InterruptStatusMsks::DIRECT_MODE_ERROR | InterruptStatusMsks::FIFO_ERROR))
    //     {
    //         disable();
    //         if (transferError != nullptr)
    //         {
    //             transferError();
    //         }
    //     }
    //     if ((currIsrs & InterruptStatusMsks::TRANSFER_COMPLETE) && transferComplete)
    //     {
    //         transferComplete();
    //     }

    //     DMA::clearInterruptFlags(STREAM_TYPEDEF_TO_STREAM_NUM(stream));
    // }

    // /**
    //  * Enable the IRQ vector of the channel
    //  *
    //  * @param[in] priority Priority of the IRQ
    //  */
    // static void enableInterruptVector(uint32_t priority = 1)
    // {
    //     NVIC_SetPriority(DmaBase::Nvic<ID>::DmaIrqs[STREAM_TYPEDEF_TO_STREAM_NUM(stream)], priority);
    //     NVIC_EnableIRQ(DmaBase::Nvic<ID>::DmaIrqs[STREAM_TYPEDEF_TO_STREAM_NUM(stream)]);
    // }

    // /**
    //  * Disable the IRQ vector of the channel
    //  */
    // static void disableInterruptVector()
    // {
    //     NVIC_DisableIRQ(DmaBase::Nvic<ID>::DmaIrqs[STREAM_TYPEDEF_TO_STREAM_NUM(stream)]);
    // }

    // /**
    //  * Set the IRQ handler for transfer errors
    //  *
    //  * The handler will be called from the channels IRQ handler function
    //  * when the IRQ status indicates an error occured.
    //  */
    // static void setTransferErrorIrqHandler(IrqHandler irqHandler) { transferError = irqHandler; }
    // /**
    //  * Set the IRQ handler for transfer complete
    //  *
    //  * Called by the channels IRQ handler when the transfer is complete.
    //  */
    // static void setTransferCompleteIrqHandler(IrqHandler irqHandler)
    // {
    //     transferComplete = irqHandler;
    // }

// private:
//     static inline DmaBase::IrqHandler transferError = nullptr;
//     static inline DmaBase::IrqHandler transferComplete = nullptr;
};  // class DmaStreamHal

}  // namespace arch
}  // namespace aruwlib

#endif  // MODM_STM32_DMA_HAL_HPP
