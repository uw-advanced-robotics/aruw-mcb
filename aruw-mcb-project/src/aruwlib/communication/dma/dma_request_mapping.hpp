/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef DMA_REQUEST_MAPPING_HPP_
#define DMA_REQUEST_MAPPING_HPP_

#include <modm/platform/dma/dma_base.hpp>  // "dma_base.hpp"

namespace modm
{
namespace platform
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

private:
    /**
     * Convert ChannelSelection into a number between [0, num channels)
     */
    static constexpr uint8_t getChannelNumber(DmaBase::ChannelSelection channel)
    {
        return uint32_t(channel) >> DMA_SxCR_CHSEL_Pos;
    }

public:
    template <uint32_t ID, DmaBase::ChannelSelection C, DmaBase::StreamID S, Peripheral P>
    static constexpr bool validateRequestMapping()
    {
        static_assert(ID == 1 || ID == 2, "invalid DMA channel");

        if constexpr (ID == 1)
        {
            return requestMapping1[getChannelNumber(C)][uint32_t(S)] == P;
        }
        else if constexpr (ID == 2)
        {
            return requestMapping2[getChannelNumber(C)][uint32_t(S)] == P;
        }
    }
};  // class DmaRequestMapping
}  // namespace platform
}  // namespace modm

#endif  // DMA_REQUEST_MAPPING_HPP_
