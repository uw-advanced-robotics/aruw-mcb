/*
 * Copyright (c) 2020, Mike Wolfram
 * Copyright (c) 2020, Matthew Arnold
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "dma.hpp"

MODM_ISR(DMA1_Stream0)
{
    using namespace modm::platform;
    Dma1::Stream<DmaBase::StreamID::STREAM_0>::interruptHandler();
}

MODM_ISR(DMA1_Stream1)
{
    using namespace modm::platform;
    Dma1::Stream<DmaBase::StreamID::STREAM_1>::interruptHandler();
}

MODM_ISR(DMA1_Stream2)
{
    using namespace modm::platform;
    Dma1::Stream<DmaBase::StreamID::STREAM_2>::interruptHandler();
}

MODM_ISR(DMA1_Stream3)
{
    using namespace modm::platform;
    Dma1::Stream<DmaBase::StreamID::STREAM_3>::interruptHandler();
}

MODM_ISR(DMA1_Stream4)
{
    using namespace modm::platform;
    Dma1::Stream<DmaBase::StreamID::STREAM_4>::interruptHandler();
}

MODM_ISR(DMA1_Stream5)
{
    using namespace modm::platform;
    Dma1::Stream<DmaBase::StreamID::STREAM_5>::interruptHandler();
}

MODM_ISR(DMA1_Stream6)
{
    using namespace modm::platform;
    Dma1::Stream<DmaBase::StreamID::STREAM_6>::interruptHandler();
}

MODM_ISR(DMA1_Stream7)
{
    using namespace modm::platform;
    Dma1::Stream<DmaBase::StreamID::STREAM_7>::interruptHandler();
}

MODM_ISR(DMA2_Stream0)
{
    using namespace modm::platform;
    Dma2::Stream<DmaBase::StreamID::STREAM_0>::interruptHandler();
}

MODM_ISR(DMA2_Stream1)
{
    using namespace modm::platform;
    Dma2::Stream<DmaBase::StreamID::STREAM_1>::interruptHandler();
}

MODM_ISR(DMA2_Stream2)
{
    using namespace modm::platform;
    Dma2::Stream<DmaBase::StreamID::STREAM_2>::interruptHandler();
}

MODM_ISR(DMA2_Stream3)
{
    using namespace modm::platform;
    Dma2::Stream<DmaBase::StreamID::STREAM_3>::interruptHandler();
}

MODM_ISR(DMA2_Stream4)
{
    using namespace modm::platform;
    Dma2::Stream<DmaBase::StreamID::STREAM_4>::interruptHandler();
}

MODM_ISR(DMA2_Stream5)
{
    using namespace modm::platform;
    Dma2::Stream<DmaBase::StreamID::STREAM_5>::interruptHandler();
}

MODM_ISR(DMA2_Stream6)
{
    using namespace modm::platform;
    Dma2::Stream<DmaBase::StreamID::STREAM_6>::interruptHandler();
}

MODM_ISR(DMA2_Stream7)
{
    using namespace modm::platform;
    Dma2::Stream<DmaBase::StreamID::STREAM_7>::interruptHandler();
}

