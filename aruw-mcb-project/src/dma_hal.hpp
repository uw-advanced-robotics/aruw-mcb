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

    /**
     * Interrupt flag, use InterruptStatusMsks enum masks to interpret the interrupt flags
     */
    static uint32_t getInterruptFlags(Stream s)
    {
        static constexpr uint32_t INTERRUPT_STREAM_BLOCK_SIZE = 6;
        int shiftOffset = 0;
        if (uint32_t(s) >= uint32_t(Stream::STREAM_4))
        {
            shiftOffset = INTERRUPT_STREAM_BLOCK_SIZE * (uint32_t(s) - uint32_t(Stream::STREAM_4));
            if (uint32_t(s) >= uint32_t(Stream::STREAM_6))
            {
                shiftOffset += 4;  // 4 bytes of padding in the middle of the register
            }

            return (DmaDef()->HISR >> shiftOffset) & INTERRUPT_STATUS_MSK;
        }
        else
        {
            shiftOffset = INTERRUPT_STREAM_BLOCK_SIZE * uint32_t(s);
            if (uint32_t(s) >= uint32_t(Stream::STREAM_2))
            {
                shiftOffset += 4;
            }
            return (DmaDef()->LISR >> shiftOffset) & INTERRUPT_STATUS_MSK;
        }
    }
};  // class DmaHal

/**
 * ID must match the DMA number for the stream
 */
template <DmaBase::Stream S, uint32_t StreamBase>
class DmaStreamHal : public DmaBase
{
public:
    static_assert(
        StreamBase == DMA1_Stream0_BASE || StreamBase == DMA2_Stream0_BASE,
        "StreamBase invalid");

#define STREAM_PTR(S, StreamBase) ((DMA_Stream_TypeDef *)StreamBase + uint32_t(S))

    /**
     * Configure the DMA stream (HAL)
     *
     * Stops the DMA stream and writes the new values to its control register.
     *
     * @param[in] channel Specifies the channel used for the specified stream.
     *                                 This parameter can be a value of @ref DMA_Channel_selection
     *
     * @param[in] direction  Specifies if the data will be transferred from memory to peripheral,
     *                                 from memory to memory or from peripheral to memory.
     *                                 This parameter can be a value of @ref
     * DMA_Data_transfer_direction
     *
     * @param[in] periphInc             Specifies whether the Peripheral address register should be
     * incremented or not This parameter can be a value of @ref DMA_Peripheral_incremented_mode
     *
     * @param[in] memInc                Specifies whether the memory address register should be
     * incremented or not. This parameter can be a value of @ref DMA_Memory_incremented_mode
     *
     * @param[in] periphDataAlignment   Specifies the Peripheral data width.
     *                                 This parameter can be a value of @ref
     * DMA_Peripheral_data_size
     *
     * @param[in] memDataAlignment;      Specifies the Memory data width.
     *                                 This parameter can be a value of @ref DMA_Memory_data_size
     *
     * @param[in] mode;                  Specifies the operation mode of the DMAy Streamx.
     *                                 This parameter can be a value of @ref DMA_mode
     *                                 @note The circular buffer mode cannot be used if the
     * memory-to-memory data transfer is configured on the selected Stream
     *
     * @param[in] priority;              Specifies the software priority for the DMAy Streamx.
     *                                 This parameter can be a value of @ref DMA_Priority_level
     *
     * @param[in] fifoMode;              Specifies if the FIFO mode or Direct mode will be used for
     * the specified stream. This parameter can be a value of @ref DMA_FIFO_direct_mode
     *                                 @note The Direct mode (FIFO mode disabled) cannot be used if
     * the memory-to-memory data transfer is configured on the selected stream
     *
     * @param[in] fifoThreshold;         Specifies the FIFO threshold level.
     *                                 This parameter can be a value of @ref
     * DMA_FIFO_threshold_level
     *
     * @param[in] memBurst;              Specifies the Burst transfer configuration for the memory
     * transfers. It specifies the amount of data to be transferred in a single non interruptible
     *                                    transaction.
     *                                      This parameter can be a value of @ref DMA_Memory_burst
     *                                      @note The burst mode is possible only if the address
     * Increment mode is enabled.
     *
     *  @param[in] periphBurst;          Specifies the Burst transfer configuration for the
     * peripheral transfers. It specifies the amount of data to be transferred in a single non
     * interruptible transaction. This parameter can be a value of @ref DMA_Peripheral_burst
     *                                      @note The burst mode is possible only if the address
     * Increment mode is enabled.
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
        disable();

        uint32_t tmp = STREAM_PTR(S, StreamBase)->CR;

        tmp &= ((uint32_t) ~(
            DMA_SxCR_CHSEL | DMA_SxCR_MBURST | DMA_SxCR_PBURST | DMA_SxCR_PL | DMA_SxCR_MSIZE |
            DMA_SxCR_PSIZE | DMA_SxCR_MINC | DMA_SxCR_PINC | DMA_SxCR_CIRC | DMA_SxCR_DIR |
            DMA_SxCR_CT | DMA_SxCR_DBM));

        tmp |= uint32_t(channel) | uint32_t(direction) | uint32_t(periphInc) |
               uint32_t(memInc) | uint32_t(peripDataSize) |
               uint32_t(memDataSize) | uint32_t(priority) |
               uint32_t(mode) | uint32_t(priority);

        if (fifoMode == FifoMode::ENABLED)
        {
            tmp |= uint32_t(memBurst) | uint32_t(periphBurst);
        }

        STREAM_PTR(S, StreamBase)->CR = tmp;

        tmp = STREAM_PTR(S, StreamBase)->FCR;

        // Clear direct mode and FIFO threshold bits
        tmp &= ~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);

        tmp |= uint32_t(fifoMode);

        if (fifoMode == FifoMode::ENABLED)
        {
            tmp |= uint32_t(fifoThreshold);

            if (memBurst != MemoryBurstTransfer::SINGLE)
            {
                // Check fifo parameters for compatibility between threshold level and size of
                // memory burst
            }
        }

        STREAM_PTR(S, StreamBase)->FCR = tmp;
    }

    static void configureDoubleBufferMode(DoubleBufferMode bufferMode)
    {
        // TODO
        STREAM_PTR(S, StreamBase)->CR |= uint32_t(bufferMode);
    }

    static void enableInterrupt(Interrupt_t irq) { STREAM_PTR(S, StreamBase)->CR |= irq.value; }

    /**
     * Enable the DMA stream to send/receive.
     */
    static void enable()
    {
        STREAM_PTR(S, StreamBase)->CR |= uint32_t(StreamEnableFlag::STREAM_ENABLED);
    }

    /**
     * Disable send/receive on the DMA stream
     */
    static void disable()
    {
        STREAM_PTR(S, StreamBase)->CR &= ~uint32_t(StreamEnableFlag::STREAM_ENABLED);
        // wait for stream to be stopped
        // TODO add timeout
        while (STREAM_PTR(S, StreamBase)->CR & uint32_t(StreamEnableFlag::STREAM_ENABLED))
            ;
    }

    static void selectChannel(ChannelSelection channel)
    {
        // int i = (~DMA_SxCR_CHSEL_Msk | uint32_t(channel));
        STREAM_PTR(S, StreamBase)->CR &= (~DMA_SxCR_CHSEL_Msk | uint32_t(channel));
    }

    static void setPeripheralIncrementMode(PeripheralIncrementMode mode)
    {
        STREAM_PTR(S, StreamBase)->CR |= (~DMA_SxCR_PINC | uint32_t(mode));
    }

    static void setMemoryIncrementMode(MemoryIncrementMode mode)
    {
        STREAM_PTR(S, StreamBase)->CR |= (~DMA_SxCR_MINC | uint32_t(mode));
    }

    /**
     * Get the direction of the data transfer
     */
    static DataTransferDirection getDataTransferDirection()
    {
        return static_cast<DataTransferDirection>(
            STREAM_PTR(S, StreamBase)->CR & (DMA_SxCR_DIR_0 | DMA_SxCR_DIR_1));
    }

    static void setSourceAddress(uintptr_t src)
    {
        if ((STREAM_PTR(S, StreamBase)->CR & uint32_t(DataTransferDirection::MEMORY_TO_MEMORY)) ==
            uint32_t(DataTransferDirection::MEMORY_TO_MEMORY))
        {
            STREAM_PTR(S, StreamBase)->PAR = src;
        }
        else if (
            (STREAM_PTR(S, StreamBase)->CR &
             uint32_t(DataTransferDirection::MEMORY_TO_PERIPHERAL)) ==
            uint32_t(DataTransferDirection::MEMORY_TO_PERIPHERAL))
        {
            STREAM_PTR(S, StreamBase)->M0AR = src;
        }
        else  // peripheral to memory
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
            (STREAM_PTR(S, StreamBase)->CR &
             uint32_t(DataTransferDirection::MEMORY_TO_PERIPHERAL)) ==
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
};  // class DmaStreamHal

}  // namespace arch
}  // namespace aruwlib

#endif  // MODM_STM32_DMA_HAL_HPP
