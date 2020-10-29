#include "dma.hpp"

MODM_ISR(DMA1_Stream0)
{
    using namespace aruwlib::arch;
    Dma1::Stream<DmaBase::Stream::STREAM_0>::interruptHandler();
}

MODM_ISR(DMA1_Stream1)
{
    using namespace aruwlib::arch;
    Dma1::Stream<DmaBase::Stream::STREAM_1>::interruptHandler();
}

MODM_ISR(DMA1_Stream2)
{
    using namespace aruwlib::arch;
    Dma1::Stream<DmaBase::Stream::STREAM_2>::interruptHandler();
}

MODM_ISR(DMA1_Stream3)
{
    using namespace aruwlib::arch;
    Dma1::Stream<DmaBase::Stream::STREAM_3>::interruptHandler();
}

MODM_ISR(DMA1_Stream4)
{
    using namespace aruwlib::arch;
    Dma1::Stream<DmaBase::Stream::STREAM_4>::interruptHandler();
}

MODM_ISR(DMA1_Stream5)
{
    using namespace aruwlib::arch;
    Dma1::Stream<DmaBase::Stream::STREAM_5>::interruptHandler();
}

MODM_ISR(DMA1_Stream6)
{
    using namespace aruwlib::arch;
    Dma1::Stream<DmaBase::Stream::STREAM_6>::interruptHandler();
}

MODM_ISR(DMA1_Stream7)
{
    using namespace aruwlib::arch;
    Dma1::Stream<DmaBase::Stream::STREAM_7>::interruptHandler();
}

MODM_ISR(DMA2_Stream0)
{
    using namespace aruwlib::arch;
    Dma2::Stream<DmaBase::Stream::STREAM_0>::interruptHandler();
}

MODM_ISR(DMA2_Stream1)
{
    using namespace aruwlib::arch;
    Dma2::Stream<DmaBase::Stream::STREAM_1>::interruptHandler();
}

MODM_ISR(DMA2_Stream2)
{
    using namespace aruwlib::arch;
    Dma2::Stream<DmaBase::Stream::STREAM_2>::interruptHandler();
}

MODM_ISR(DMA2_Stream3)
{
    using namespace aruwlib::arch;
    Dma2::Stream<DmaBase::Stream::STREAM_3>::interruptHandler();
}

MODM_ISR(DMA2_Stream4)
{
    using namespace aruwlib::arch;
    Dma2::Stream<DmaBase::Stream::STREAM_4>::interruptHandler();
}

MODM_ISR(DMA2_Stream5)
{
    using namespace aruwlib::arch;
    Dma2::Stream<DmaBase::Stream::STREAM_5>::interruptHandler();
}

MODM_ISR(DMA2_Stream6)
{
    using namespace aruwlib::arch;
    Dma2::Stream<DmaBase::Stream::STREAM_6>::interruptHandler();
}

MODM_ISR(DMA2_Stream7)
{
    using namespace aruwlib::arch;
    Dma2::Stream<DmaBase::Stream::STREAM_7>::interruptHandler();
}
