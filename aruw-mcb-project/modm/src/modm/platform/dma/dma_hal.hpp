/*
 * Copyright (c) 2020, Matthew Arnold
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef DMA_HAL_HPP_
#define DMA_HAL_HPP_

#include <modm/architecture/interface/assert.hpp>
#include <modm/architecture/interface/clock.hpp>

#include "dma_base.hpp"

namespace modm
{
namespace platform
{
template <uint32_t ID>
class DmaHal : public DmaBase
{
public:
    static_assert(ID >= 1 && ID <= 2, "dma id invalid");

    static constexpr uint32_t streamBase()
    {
        if constexpr (ID == 1)
        {
            return DMA1_Stream0_BASE;
        }
        else if constexpr (ID == 2)
        {
            return DMA2_Stream0_BASE;
        }
    }

    static constexpr DMA_TypeDef *dmaDef()
    {
        if constexpr (ID == 1)
        {
            return DMA1;
        }
        else if constexpr (ID == 2)
        {
            return DMA2;
        }
    }

    /**
     * Sets the bits associated with the stream's interrupt flag clear register to high
     * (either LIFCR or HIFCR).
     */
    static void clearInterruptFlags(StreamID s)
    {
        if (uint32_t(s) > uint32_t(StreamID::STREAM_3))
        {
            dmaDef()->HIFCR |= dmaStreamIFCRMasks[uint32_t(s)];
        }
        else
        {
            dmaDef()->LIFCR |= dmaStreamIFCRMasks[uint32_t(s)];
        }
    }

    /**
     * Interrupt flag, use InterruptStatusMsks enum masks to interpret the interrupt flags
     */
    static uint32_t getInterruptFlags(StreamID s)
    {
        static constexpr uint32_t INTERRUPT_STREAM_BLOCK_SIZE = 6;
        int shiftOffset = 0;
        if (uint32_t(s) >= uint32_t(StreamID::STREAM_4))
        {
            shiftOffset =
                INTERRUPT_STREAM_BLOCK_SIZE * (uint32_t(s) - uint32_t(StreamID::STREAM_4));
            if (uint32_t(s) >= uint32_t(StreamID::STREAM_6))
            {
                shiftOffset += 4;  // 4 bytes of padding in the middle of the register
            }

            return (dmaDef()->HISR >> shiftOffset) & INTERRUPT_STATUS_MSK;
        }
        else
        {
            shiftOffset = INTERRUPT_STREAM_BLOCK_SIZE * uint32_t(s);
            if (uint32_t(s) >= uint32_t(StreamID::STREAM_2))
            {
                shiftOffset += 4;
            }
            return (dmaDef()->LISR >> shiftOffset) & INTERRUPT_STATUS_MSK;
        }
    }
};  // class DmaHal

/**
 * Hardware abstraction of a DMA stream.
 *
 * TODO
 * - Double buffering mode
 *
 * @note ID must match the DMA number for the stream
 */
template <DmaBase::StreamID S, uint32_t StreamBase>
class DmaStreamHal : public DmaBase
{
    static inline DMA_Stream_TypeDef *streamPtr()
    {
        return reinterpret_cast<DMA_Stream_TypeDef *>(StreamBase) + uint32_t(S);
    }

    static constexpr int32_t DMA_DISABLE_TIMEOUT = 2;

public:
    static_assert(
        StreamBase == DMA1_Stream0_BASE ||
        StreamBase == DMA2_Stream0_BASE ||
        true
        "StreamBase invalid");

    /**
     * Configure the DMA stream.
     *
     * Stops the DMA stream and writes the new values to its control register.
     *
     * @param[in] channel Specifies the channel used for the specified stream.
     * @param[in] direction Specifies if the data will be transferred from
     *      memory to peripheral, from memory to memory or from peripheral to
     *      memory.
     * @param[in] periphInc Specifies whether the peripheral address register
     *      should be incremented or not.
     * @param[in] memInc Specifies whether the memory address register should
     *      be incremented or not.
     * @param[in] periphDataAlignment Specifies the Peripheral data width.
     * @param[in] memDataAlignment Specifies the Memory data width.
     * @param[in] mode Specifies the operation mode of the DMAy Streamx.
     * @param[in] priority Specifies the software priority for the DMA ID y
     *      Stream x.
     * @param[in] fifoMode Specifies if the FIFO mode or Direct mode will be
     *      used for the specified stream.
     * @param[in] fifoThreshold Specifies the FIFO threshold level.
     * @param[in] memBurst Specifies the Burst transfer configuration for the
     *      memory transfers. It specifies the amount of data to be transferred
     *      in a single non interruptible transaction.
     * @param[in] periphBurst Specifies the burst transfer configuration for the
     *      peripheral transfers. It specifies the amount of data to be
     *      transferred in a single non interruptible transaction.
     *
     * @note The burst mode is possible only if the address increment mode is enabled.
     * @note The circular buffer mode cannot be used if the memory-to-memory
     *      data transfer is configured on the selected stream.
     * @note The Direct mode (FIFO mode disabled) cannot be used if the
     *      memory-to-memory data transfer is configured on the selected stream
     * @note The burst mode is possible only if the address increment mode is enabled.
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

        uint32_t tmp = streamPtr()->CR;

        tmp &= ((uint32_t) ~(
            DMA_SxCR_CHSEL | DMA_SxCR_MBURST | DMA_SxCR_PBURST |
            DMA_SxCR_PL    | DMA_SxCR_MSIZE  | DMA_SxCR_PSIZE  |
            DMA_SxCR_MINC  | DMA_SxCR_PINC   | DMA_SxCR_CIRC   |
            DMA_SxCR_DIR   | DMA_SxCR_CT     | DMA_SxCR_DBM));

        tmp |= uint32_t(channel)  | uint32_t(direction)     | uint32_t(periphInc)   |
               uint32_t(memInc)   | uint32_t(peripDataSize) | uint32_t(memDataSize) |
               uint32_t(priority) | uint32_t(mode)          | uint32_t(priority);

        if (fifoMode == FifoMode::ENABLED)
        {
            tmp |= uint32_t(memBurst) | uint32_t(periphBurst);
        }

        streamPtr()->CR = tmp;

        tmp = streamPtr()->FCR;

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
                // TODO
            }
        }

        streamPtr()->FCR = tmp;
    }

    static void configureDoubleBufferMode(DoubleBufferMode)
    {
        // TODO
    }

    static void enableInterrupt(Interrupt_t irq) { streamPtr()->CR |= irq.value; }

    static void disableInterrupt(Interrupt_t irq) { streamPtr()->CR &= ~irq.value; }

    /**
     * Enable the DMA stream to send/receive.
     */
    static void enable() { streamPtr()->CR |= uint32_t(StreamEnableFlag::STREAM_ENABLED); }

    /**
     * Disable send/receive on the DMA stream
     */
    static bool disable()
    {
        streamPtr()->CR &= ~uint32_t(StreamEnableFlag::STREAM_ENABLED);
        // wait for stream to be stopped
        uint32_t start = modm::Clock::now().time_since_epoch().count();
        while ((streamPtr()->CR & uint32_t(StreamEnableFlag::STREAM_ENABLED)) &&
               modm::Clock::now().time_since_epoch().count() - start > DMA_DISABLE_TIMEOUT)
            ;
        if (streamPtr()->CR & uint32_t(StreamEnableFlag::STREAM_ENABLED))
        {
            return false;
        }
        return true;
    }

    /**
     * Get the direction of the data transfer
     */
    static DataTransferDirection getDataTransferDirection()
    {
        return static_cast<DataTransferDirection>(
            streamPtr()->CR & (DMA_SxCR_DIR_0 | DMA_SxCR_DIR_1));
    }

    /**
     * Sets the sources address register based on if the transfer mode is memory to peripheral,
     * peripheral to memory, or memory to memory.
     */
    static void setSourceAddress(uintptr_t src)
    {
        if ((streamPtr()->CR & uint32_t(DataTransferDirection::MEM_TO_MEM)) ==
            uint32_t(DataTransferDirection::MEM_TO_MEM))
        {
            streamPtr()->PAR = src;
        }
        else if (
            (streamPtr()->CR & uint32_t(DataTransferDirection::MEM_TO_PERIPH)) ==
            uint32_t(DataTransferDirection::MEM_TO_PERIPH))
        {
            streamPtr()->M0AR = src;
        }
        else  // peripheral to memory
        {
            streamPtr()->PAR = src;
        }
    }

    /**
     * Sets the destination address register based on if the transfer mode is memory to peripheral,
     * peripheral to memory, or memory to memory.
     */
    static void setDestinationAddress(uintptr_t dst)
    {
        if ((streamPtr()->CR & uint32_t(DataTransferDirection::MEM_TO_MEM)) ==
            uint32_t(DataTransferDirection::MEM_TO_MEM))
        {
            streamPtr()->M0AR = dst;
        }
        else if (
            (streamPtr()->CR & uint32_t(DataTransferDirection::MEM_TO_PERIPH)) ==
            uint32_t(DataTransferDirection::MEM_TO_PERIPH))
        {
            streamPtr()->PAR = dst;
        }
        else
        {
            streamPtr()->M0AR = dst;
        }
    }

    static void setDataLength(std::size_t length) { streamPtr()->NDTR = length; }
};  // class DmaStreamHal

}  // namespace platform
}  // namespace modm

#endif  // MODM_STM32_DMA_HAL_HPP
