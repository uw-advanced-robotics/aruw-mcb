#ifndef UART_1_DMA_HPP_
#define UART_1_DMA_HPP_

#include <modm/platform/uart/uart_1.hpp>

namespace aruwlib
{
namespace arch
{
template <typename DmaStreamRx, typename DmaStreamTx>
class Usart1Dma : modm::platform::Usart1
{
public:
    static void read(uint8_t *rxBuff, std::size_t length)
    {
        if (rxBuff == nullptr || length < 0)
        {
            return;
        }
        DmaStreamRx::setSourceAddress(rxBuff);
        DmaStreamRx::setDataLength(length);
        DmaStreamRx::enable();
    }

private:
};  // class Usart1Dma
}  // namespace arch
}  // namespace aruwlib

#endif  // UART_1_DMA_HPP_
