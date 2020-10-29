#ifndef MODM_STM32_DMA_BASE_HPP
#define MODM_STM32_DMA_BASE_HPP

#include <stdint.h>

#include <modm/architecture/interface/assert.hpp>
#include <modm/architecture/interface/interrupt.hpp>
#include <modm/architecture/interface/register.hpp>
#include <modm/platform/device.hpp>

namespace aruwlib
{
namespace arch
{
/**
 * Class that defines register masks and interrupt handlers for DMA.
 */
class DmaBase
{
public:
    // Enums, for register bit masking

    // Use the following bit masks on a DMA stream x configuration register
    // DMA_xCR, where x = [0,7]

    // Bits 27:25
    enum class ChannelSelection : uint32_t
    {
        CHANNEL_0 = 0,
        CHANNEL_1 = DMA_SxCR_CHSEL_0,
        CHANNEL_2 = DMA_SxCR_CHSEL_1,
        CHANNEL_3 = DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0,
        CHANNEL_4 = DMA_SxCR_CHSEL_2,
        CHANNEL_5 = DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_0,
        CHANNEL_6 = DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_1,
        CHANNEL_7 = DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0,
    };

    // Bits 24:23
    enum class MemoryBurstTransfer : uint32_t
    {
        SINGLE = 0,
        INCREMENT_4 = DMA_SxCR_MBURST_0,
        INCREMENT_8 = DMA_SxCR_MBURST_1,
        INCREMENT_16 = DMA_SxCR_MBURST_1 | DMA_SxCR_MBURST_0,
    };

    // Bits 22:21
    enum class PeripheralBurstTransfer : uint32_t
    {
        SINGLE = 0,
        INCREMENT_4 = DMA_SxCR_PBURST_0,
        INCREMENT_8 = DMA_SxCR_PBURST_1,
        INCREMENT_16 = DMA_SxCR_PBURST_1 | DMA_SxCR_PBURST_0,
    };

    // Bit 19
    // Only used in double buffer mode
    enum class CurrentTarget : uint32_t
    {
        MEMORY_0 = 0,
        MEMORY_1 = DMA_SxCR_CT,
    };

    // Bit 18
    enum class DoubleBufferMode : uint32_t
    {
        NO_BUFFER_SWITCHING = 0,
        MEMORY_TARGET_SWITCHED = DMA_SxCR_DBM,
    };

    // Bits 17:16
    enum class PriorityLevel : uint32_t
    {
        LOW = 0,
        MEDIUM = DMA_SxCR_PL_0,
        HIGH = DMA_SxCR_PL_1,
        VERY_HIGH = DMA_SxCR_PL_0 | DMA_SxCR_PL_1,
    };

    // Bit 15
    enum class PeripheralIncrementOffsetSize : uint32_t
    {
        LINKED_TO_PSIZE = 0,
        FIXED_TO_4 = DMA_SxCR_PINC,
    };

    // Bits 14:13
    /// In direct mode (if the FIFO is not used)
    /// MSIZE is forced by hardware to the same value as PSIZE
    enum class MemoryDataSize : uint32_t
    {
        BYTE = 0,
        HALF_WORD = DMA_SxCR_MSIZE_0,
        WORD = DMA_SxCR_MSIZE_1,
    };

    // Bits 12:11
    enum class PeripheralDataSize : uint32_t
    {
        BYTE = 0,
        HALF_WORD = DMA_SxCR_PSIZE_0,
        WORD = DMA_SxCR_PSIZE_1,
    };

    // Bit 10
    enum class MemoryIncrementMode : uint32_t
    {
        FIXED = 0,
        INCREMENT = DMA_SxCR_MINC,  ///< incremented according to MemoryDataSize
    };

    // Bit 9
    enum class PeripheralIncrementMode : uint32_t
    {
        FIXED = 0,
        INCREMENT = DMA_SxCR_PINC,  ///< incremented according to PeripheralDataSize
    };

    // Bit 8
    enum class CircularMode : uint32_t
    {
        DISABLED = 0,
        ENABLED = DMA_SxCR_CIRC,  ///< circular mode
    };

    // Bits 7:6
    enum class DataTransferDirection : uint32_t
    {
        ///< Source: DMA_SxPAR; Sink: DMA_SxM0AR
        PERIPHERAL_TO_MEMORY = 0,
        ///< Source: DMA_SxM0AR; Sink: DMA_SxPAR
        MEMORY_TO_PERIPHERAL = DMA_SxCR_DIR_0,
        ///< Source: DMA_SxPAR; Sink: DMA_SxM0AR
        MEMORY_TO_MEMORY = DMA_SxCR_DIR_1,
    };

    // Bit 5
    enum class FlowControl : uint32_t
    {
        DMA = 0,
        PERIPHERAL = DMA_SxCR_PFCTRL,  ///< the peripheral is the flow controller
    };

    // Bit 1-4 are interrupt enable settings.
    enum class Interrupt
    {
        TRANSFER_COMPLETE = DMA_SxCR_TCIE,
        HALF_TRANSFER_COMPLETE = DMA_SxCR_HTIE,
        ERROR = DMA_SxCR_DMEIE,
        ALL = DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_DMEIE,
    };
    MODM_FLAGS32(Interrupt);

    /**
     * Bit 0
     *
     * When disabled, software allowed to program the Configuration and FIFO
     * bit registers. It is forbidden to write these registers when EN bit is
     * read as 1.
     * @note Before setting EN bit to '1' to start a new transfer, the event
     *      flags corresponding to the stream in DMA_LISR or DMA_HISR register
     *      must be cleared.
     */
    enum class StreamEnableFlag : uint32_t
    {
        STREAM_DISABLED = 0,
        STREAM_ENABLED = DMA_SxCR_EN,
    };

    // End DMA_xCR masks

#define STREAM_TYPEDEF_TO_STREAM_NUM(stream)                                    \
    (stream > DMA1_Stream7)                                                     \
        ? ((uint64_t)(stream - DMA2_Stream0_BASE) / sizeof(DMA_Stream_TypeDef)) \
        : ((uint64_t)(stream - DMA1_Stream0_BASE) / sizeof(DMA_Stream_TypeDef))

    enum class Stream : uint32_t
    {
        STREAM_0 = 0,
        STREAM_1 = 1,
        STREAM_2 = 2,
        STREAM_3 = 3,
        STREAM_4 = 4,
        STREAM_5 = 5,
        STREAM_6 = 6,
        STREAM_7 = 7,
    };

    static constexpr uint8_t getChannelNumber(ChannelSelection channel)
    {
        return uint32_t(channel) >> DMA_SxCR_CHSEL_Pos;
    }

protected:
    /**
     * Use to clear interrupts on a particular stream.
     */
    static constexpr uint32_t dmaStreamIFCRMasks[]{
        DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CHTIF0 |
            DMA_LIFCR_CTCIF0,
        DMA_LIFCR_CFEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CHTIF1 |
            DMA_LIFCR_CTCIF1,
        DMA_LIFCR_CFEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CTEIF2 | DMA_LIFCR_CHTIF2 |
            DMA_LIFCR_CTCIF2,
        DMA_LIFCR_CFEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3 |
            DMA_LIFCR_CTCIF3,
        DMA_HIFCR_CFEIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CTEIF4 | DMA_HIFCR_CHTIF4 |
            DMA_HIFCR_CTCIF4,
        DMA_HIFCR_CFEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CHTIF5 |
            DMA_HIFCR_CTCIF5,
        DMA_HIFCR_CFEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CHTIF6 |
            DMA_HIFCR_CTCIF6,
        DMA_HIFCR_CFEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CHTIF7 |
            DMA_HIFCR_CTCIF7,
    };

    static constexpr uint32_t INTERRUPT_STATUS_MSK =
        DMA_LISR_TCIF0 | DMA_LISR_HTIF0 | DMA_LISR_TEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_FEIF0;

    enum class InterruptStatusMsks : uint32_t
    {
        TRANSFER_COMPLETE = DMA_LISR_TCIF0,
        HALF_TRANSFER_COMPLETE = DMA_LISR_HTIF0,
        TRANSFER_ERROR = DMA_LISR_TEIF0,
        DIRECT_MODE_ERROR = DMA_LISR_DMEIF0,
        FIFO_ERROR = DMA_LISR_FEIF0,
    };

    using IrqHandler = void (*)(void);

    /**
     * Defines a list of interrupt vectors to be enabled for each DMA stream.
     *
     * @tparam The DMA identifier
     */
    template <uint32_t ID>
    struct Nvic;
};  // class DmaBase

template <>
struct DmaBase::Nvic<1>
{
    static constexpr IRQn_Type DmaIrqs[]{
        DMA1_Stream1_IRQn,
        DMA1_Stream2_IRQn,
        DMA1_Stream3_IRQn,
        DMA1_Stream4_IRQn,
        DMA1_Stream5_IRQn,
        DMA1_Stream6_IRQn,
        DMA1_Stream7_IRQn,
    };
};  // struct Nvic<1>

template <>
struct DmaBase::Nvic<2>
{
    static constexpr IRQn_Type DmaIrqs[]{
        DMA2_Stream1_IRQn,
        DMA2_Stream2_IRQn,
        DMA2_Stream3_IRQn,
        DMA2_Stream4_IRQn,
        DMA2_Stream5_IRQn,
        DMA2_Stream6_IRQn,
        DMA2_Stream7_IRQn,
    };
};  // struct Nvic<2>
}  // namespace arch
}  // namespace aruwlib

#endif  // MODM_STM32_DMA_BASE_HPP
