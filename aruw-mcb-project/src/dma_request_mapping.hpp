#ifndef DMA_REQUEST_MAPPING_HPP_
#define DMA_REQUEST_MAPPING_HPP_

#include "dma_base.hpp"

namespace aruwlib
{
namespace arch
{
class DmaRequestMapping
{
public:
    enum class Peripheral
    {
        USART1_TX,
        USART1_RX,
        INVALID,
    };

    static constexpr Peripheral requestMapping1[][8]{
        {Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID},
        {Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID},
        {Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID},
        {Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID},
        {Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID},
        {Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID},
        {Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID},
        {Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID},
    };

    static constexpr Peripheral requestMapping2[][8]{
        {Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID},
        {Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID},
        {Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID},
        {Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID},
        {Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::USART1_RX,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::USART1_TX},
        {Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID},
        {Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID},
        {Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID,
         Peripheral::INVALID},
    };

    template <uint32_t ID, DmaBase::ChannelSelection C, DmaBase::Stream S, Peripheral P>
    static constexpr bool validateRequestMapping()
    {
        static_assert(ID == 1 || ID == 2, "invalid DMA channel");

        if constexpr (ID == 1)
        {
            return requestMapping1[DmaBase::getChannelNumber(C)][uint32_t(S)] == P;
        }
        else if constexpr (ID == 2)
        {
            return requestMapping2[DmaBase::getChannelNumber(C)][uint32_t(S)] == P;
        }
    }
};  // class DmaRequestMapping
}  // namespace arch
}  // namespace aruwlib

#endif  // DMA_REQUEST_MAPPING_HPP_
