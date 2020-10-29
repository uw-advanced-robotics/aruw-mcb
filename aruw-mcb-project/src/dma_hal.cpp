// #include "dma_hal.hpp"

// MODM_ISR(DMA{{ streams.instance }}_Stream{{ stream.position }})
// {
// 	using namespace modm::platform;
// 	Dma1::Channel<DmaBase::Channel::Channel{{ stream.position }}>::interruptHandler();
// }