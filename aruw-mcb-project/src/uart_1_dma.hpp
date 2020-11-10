#ifndef UART_1_DMA_HPP_
#define UART_1_DMA_HPP_

#include <modm/architecture/interface/uart.hpp>
#include <modm/platform/gpio/connector.hpp>
#include <modm/platform/uart/uart_hal_1.hpp>

#include "dma.hpp"
#include "dma_request_mapping.hpp"

// __weak void uart1DmaMessageReceived(void);

namespace aruwlib
{
namespace arch
{
/**
 * Interrupt handler callback for USART1_isr.
 */
__attribute__((weak)) void uart1IrqHandler(void);
/**
 * Interrupt handlers for USART1 DMA TX and RX streams.
 */
__attribute__((weak)) void uart1DmaTxComplete(void);
__attribute__((weak)) void uart1DmaTxError(void);
__attribute__((weak)) void uart1DmaRxComplete(void);
__attribute__((weak)) void uart1DmaRxError(void);

template <
    typename DmaStreamTx,
    DmaBase::ChannelSelection ChannelIdRx,
    typename DmaStreamRx,
    DmaBase::ChannelSelection ChannelIdTx>
class Usart1Dma : public modm::platform::UartBase, public modm::Uart
{
private:
    static_assert(
        DmaRequestMapping::validateRequestMapping<
            DmaStreamRx::DMA_ID,
            ChannelIdRx,
            DmaStreamRx::STREAM_ID,
            DmaRequestMapping::Peripheral::USART1_RX>(),
        "dma usart1 tx channel or stream selection invalid");

    static_assert(
        DmaRequestMapping::validateRequestMapping<
            DmaStreamTx::DMA_ID,
            ChannelIdTx,
            DmaStreamTx::STREAM_ID,
            DmaRequestMapping::Peripheral::USART1_TX>(),
        "dma usart1 tx channel or stream selection invalid");

public:
    template <template <modm::platform::Peripheral _> class... Signals>
    static void connect(
        modm::platform::Gpio::InputType InputTypeRx = modm::platform::Gpio::InputType::PullUp,
        modm::platform::Gpio::OutputType OutputTypeTx = modm::platform::Gpio::OutputType::PushPull)
    {
        using Connector =
            modm::platform::GpioConnector<modm::platform::Peripheral::Usart1, Signals...>;
        using Tx = typename Connector::template GetSignal<modm::platform::Gpio::Signal::Tx>;
        using Rx = typename Connector::template GetSignal<modm::platform::Gpio::Signal::Rx>;
        static_assert(
            ((Connector::template IsValid<Tx> and
              Connector::template IsValid<Rx>)and sizeof...(Signals) == 2) or
                ((Connector::template IsValid<Tx> or
                  Connector::template IsValid<Rx>)and sizeof...(Signals) == 1),
            "Usart1::connect() requires one Tx and/or one Rx signal!");

        Tx::setOutput(OutputTypeTx);
        Rx::setInput(InputTypeRx);
        Connector::connect();
    }

    /**
     * Performs DMA/UART specific integration configuration.
     *
     * @note DmaStreamRx and DmaStreamTx should be pre-configured (configuration is not done in
     *      this function). Also, this function **does not** configure any interrupts. The user
     *      may choose to do this.
     */
    template <
        typename SystemClock,
        modm::baudrate_t baudrate,
        modm::percent_t tolerance = modm::pct(1)>
    static void initialize(
        bool enableDmaInterrupts = true,
        uint32_t interruptPriority = 12,
        modm::platform::UartBase::Parity parity = Parity::Disabled)
    {
        modm::platform::UsartHal1::initializeWithBrr(
            modm::platform::UartBaudrate::getBrr<SystemClock::Usart1, baudrate, tolerance>(),
            parity,
            modm::platform::UartBaudrate::getOversamplingMode(SystemClock::Usart1, baudrate));

        modm::platform::UsartHal1::enableInterruptVector(true, interruptPriority);
        modm::platform::UsartHal1::enableInterrupt(Interrupt::RxNotEmpty);
        modm::platform::UsartHal1::setTransmitterEnable(true);
        modm::platform::UsartHal1::setReceiverEnable(true);

        // See page 1003 in STM32F405/415, STM32F407/417, STM32F427/437 and STM32F429/439 advanced
        // Arm-based 32-bit MCUs - Reference manual
        DmaStreamTx::disable();

        // Write the memory address in the DMA configuration register to configure it as the
        // source of the transfer. The data will be loaded into the USART_DR register from this
        // memory area after each TXE event.
        DmaStreamTx::setDestinationAddress(reinterpret_cast<uintptr_t>(&USART1->DR));

        if (enableDmaInterrupts)
        {
            DmaStreamTx::enableInterruptVector(interruptPriority);
            DmaStreamTx::enableInterrupt(DmaBase::Interrupt::ALL);
            DmaStreamTx::setTransferErrorIrqHandler(transferErrorHandlerTx);
            DmaStreamTx::setTransferCompleteIrqHandler(transferCompleteHandlerTx);
        }

        // Clear the TC bit in the SR register by writing 0 to it.
        USART1->SR &= ~USART_SR_TC;

        // Enable UART DMA TX
        USART1->CR3 |= USART_CR3_DMAT;

        // Write the USART_DR register address in the DMA control register to configure it as the
        // source of the transfer. The data will be moved from this address to the memory after
        // each RXNE event.
        DmaStreamRx::setSourceAddress(reinterpret_cast<uintptr_t>(&USART1->DR));

        if (enableDmaInterrupts)
        {
            DmaStreamRx::enableInterruptVector(interruptPriority);
            DmaStreamRx::enableInterrupt(DmaBase::Interrupt::ALL);
            DmaStreamTx::setTransferErrorIrqHandler(transferErrorHandlerRx);
            DmaStreamTx::setTransferCompleteIrqHandler(transferCompleteHandlerRx);
        }

        // Enable UART DMA RX
        USART1->CR3 |= USART_CR3_DMAR;
    }

    static void flushWriteBuffer()
    {
        while (!isWriteFinished())
            ;
    }

    static bool write(uint8_t &data)
    {
        finishedTx = false;
        return DmaStreamTx::configureWrite(&data);
    }

    static bool write(const uint8_t *buffer, std::size_t length)
    {
        finishedTx = false;
        return DmaStreamTx::configureWrite(buffer, length);
    }

    /**
     * @note Write interrupts must be enabled on the tx channel (pass in `enableDmaInterrupts =
     * true` into `initialize` in order for this function to work, otherwise `true` is always
     *      returned.
     */
    static bool isWriteFinished()
    {
        if (DmaStreamRx::getInterruptFlags().any(DmaBase::Interrupt::TRANSFER_COMPLETE))
        {
            return finishedTx;
        }
        else
        {
            return false;
        }
    }

    static bool read(uint8_t &data) { return DmaStreamRx::configureRead(&data, 1); }

    static bool read(uint8_t *buffer, std::size_t length)
    {
        return DmaStreamRx::configureRead(buffer, length);
    }

    static bool hasError()
    {
        return modm::platform::UsartHal1::getInterruptFlags().any(
            modm::platform::UsartHal1::InterruptFlag::ParityError |
#ifdef USART_ISR_NE
            modm::platform::UsartHal1::InterruptFlag::NoiseError |
#endif
            modm::platform::UsartHal1::InterruptFlag::OverrunError |
            modm::platform::UsartHal1::InterruptFlag::FramingError);
    }

    static void clearError()
    {
        return modm::platform::UsartHal1::acknowledgeInterruptFlags(
            modm::platform::UsartHal1::InterruptFlag::ParityError |
#ifdef USART_ISR_NE
            modm::platform::UsartHal1::InterruptFlag::NoiseError |
#endif
            modm::platform::UsartHal1::InterruptFlag::OverrunError |
            modm::platform::UsartHal1::InterruptFlag::FramingError);
    }

    static void clearIdleFlag()
    {
        int val = 0;
        val = USART1->SR;
        val = USART1->DR;
        static_cast<void>(val);
    }

    static void transferErrorHandlerTx()
    {
        DmaStreamTx::disable();
        hasErrorTx = true;
        uart1DmaTxError();
    }

    static void transferCompleteHandlerTx()
    {
        // When number of data transfers programmed in the DMA controller is reached, the DMA
        // controller generates an interrupt on the DMA channel interrupt vector The DMAR bit should
        // be cleared by software in the USART_Cr3 register during the interrupt subroutine
        DmaStreamRx::disable();
        USART1->CR3 &= ~USART_CR3_DMAR;
        finishedTx = true;
        uart1DmaTxComplete();
    }

    static void transferErrorHandlerRx()
    {
        DmaStreamRx::disable();
        uart1DmaRxError();
    }

    static void transferCompleteHandlerRx()
    {
        DmaStreamRx::disable();
        uart1DmaRxComplete();
    }

private:
    static inline volatile bool finishedTx = false;
    static inline volatile bool hasErrorTx = false;
};  // class Usart1Dma
}  // namespace arch
}  // namespace aruwlib

#endif  // UART_1_DMA_HPP_
