/*
 * Copyright (c) 2020 Matthew Arnold
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_UART_1_DMA_HPP
#define MODM_STM32_UART_1_DMA_HPP

#include <modm/architecture/interface/uart.hpp>
#include <modm/platform/gpio/connector.hpp>
#include "uart_hal_1.hpp"

#include <modm/platform/dma/dma.hpp>

namespace modm
{
namespace platform
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
class Usart1 : public UartBase, public ::modm::Uart
{
private:
    using TxStream = typename DmaStreamTx::template RequestMapping<ChannelIdTx, Peripheral::Usart1, DmaBase::Signal::Tx>::Stream;
    using RxStream = typename DmaStreamRx::template RequestMapping<ChannelIdRx, Peripheral::Usart1, DmaBase::Signal::Rx>::Stream;

public:
    template <template <Peripheral _> class... Signals>
    static void connect(Gpio::InputType InputTypeRx = Gpio::InputType::PullUp,
                        Gpio::OutputType OutputTypeTx = Gpio::OutputType::PushPull)
    {
        using Connector = GpioConnector<Peripheral::Usart1, Signals...>;
        using Tx = typename Connector::template GetSignal<Gpio::Signal::Tx>;
        using Rx = typename Connector::template GetSignal<Gpio::Signal::Rx>;
        static_assert(((Connector::template IsValid<Tx> and Connector::template IsValid<Rx>)and sizeof...(Signals) == 2) or
                      ((Connector::template IsValid<Tx> or Connector::template IsValid<Rx>)and sizeof...(Signals) == 1),
                      "Usart1::connect() requires one Tx and/or one Rx signal!");

        Tx::setOutput(OutputTypeTx);
        Rx::setInput(InputTypeRx);
        Connector::connect();
    }

    /**
     * Performs DMA/UART specific integration configuration.
     *
     * @note RxStream and TxStream should be pre-configured (configuration is not done in
     *      this function). Also, this function **does not** configure any interrupts. The user
     *      may choose to do this.
     */
    template <typename SystemClock, modm::baudrate_t baudrate, modm::percent_t tolerance = modm::pct(1)>
    static void initialize(bool enableDmaInterrupts = true, uint32_t interruptPriority = 12,
                           UartBase::Parity parity = Parity::Disabled)
    {
        UsartHal1::initializeWithBrr(
            UartBaudrate::getBrr<SystemClock::Usart1, baudrate, tolerance>(),
            parity, UartBaudrate::getOversamplingMode(SystemClock::Usart1, baudrate));

        UsartHal1::enableInterruptVector(true, interruptPriority);
        UsartHal1::enableInterrupt(Interrupt::RxNotEmpty);
        UsartHal1::setTransmitterEnable(true);
        UsartHal1::setReceiverEnable(true);

        // See page 1003 in STM32F405/415, STM32F407/417, STM32F427/437 and STM32F429/439 advanced
        // Arm-based 32-bit MCUs - Reference manual
        TxStream::disable();

        // Write the memory address in the DMA configuration register to configure it as the
        // source of the transfer. The data will be loaded into the USART_DR register from this
        // memory area after each TXE event.
        TxStream::setDestinationAddress(reinterpret_cast<uintptr_t>(&USART1->DR));

        if (enableDmaInterrupts)
        {
            TxStream::enableInterruptVector(interruptPriority);
            TxStream::enableInterrupt(DmaBase::Interrupt::ALL);
            TxStream::setTransferErrorIrqHandler(transferErrorHandlerTx);
            TxStream::setTransferCompleteIrqHandler(transferCompleteHandlerTx);
        }

        // Clear the TC bit in the SR register by writing 0 to it.
        USART1->SR &= ~USART_SR_TC;

        // Enable UART DMA TX
        USART1->CR3 |= USART_CR3_DMAT;

        // Write the USART_DR register address in the DMA control register to configure it as the
        // source of the transfer. The data will be moved from this address to the memory after
        // each RXNE event.
        RxStream::setSourceAddress(reinterpret_cast<uintptr_t>(&USART1->DR));

        if (enableDmaInterrupts)
        {
            RxStream::enableInterruptVector(interruptPriority);
            RxStream::enableInterrupt(DmaBase::Interrupt::ALL);
            TxStream::setTransferErrorIrqHandler(transferErrorHandlerRx);
            TxStream::setTransferCompleteIrqHandler(transferCompleteHandlerRx);
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
        return TxStream::configureWrite(&data);
    }

    static bool write(const uint8_t *buffer, std::size_t length)
    {
        finishedTx = false;
        return TxStream::configureWrite(buffer, length);
    }

    /**
     * @note Write interrupts must be enabled on the tx channel (pass in `enableDmaInterrupts =
     * true` into `initialize` in order for this function to work, otherwise `true` is always
     *      returned.
     */
    static bool isWriteFinished()
    {
        if (RxStream::getInterruptFlags().any(DmaBase::Interrupt::TRANSFER_COMPLETE))
        {
            return finishedTx;
        }
        else
        {
            return false;
        }
    }

    static bool read(uint8_t &data) { return RxStream::configureRead(&data, 1); }

    static bool read(uint8_t *buffer, std::size_t length)
    {
        return RxStream::configureRead(buffer, length);
    }

    static bool hasError()
    {
        return UsartHal1::getInterruptFlags().any(
            UsartHal1::InterruptFlag::ParityError |
#ifdef USART_ISR_NE
            UsartHal1::InterruptFlag::NoiseError |
#endif
            UsartHal1::InterruptFlag::OverrunError |
            UsartHal1::InterruptFlag::FramingError);
    }

    static void clearError()
    {
        return UsartHal1::acknowledgeInterruptFlags(
            UsartHal1::InterruptFlag::ParityError |
#ifdef USART_ISR_NE
            UsartHal1::InterruptFlag::NoiseError |
#endif
            UsartHal1::InterruptFlag::OverrunError |
            UsartHal1::InterruptFlag::FramingError);
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
        TxStream::disable();
        hasErrorTx = true;
        uart1DmaTxError();
    }

    static void transferCompleteHandlerTx()
    {
        // When number of data transfers programmed in the DMA controller is reached, the DMA
        // controller generates an interrupt on the DMA channel interrupt vector The DMAR bit should
        // be cleared by software in the USART_Cr3 register during the interrupt subroutine
        RxStream::disable();
        USART1->CR3 &= ~USART_CR3_DMAR;
        finishedTx = true;
        uart1DmaTxComplete();
    }

    static void transferErrorHandlerRx()
    {
        RxStream::disable();
        uart1DmaRxError();
    }

    static void transferCompleteHandlerRx()
    {
        RxStream::disable();
        uart1DmaRxComplete();
    }

private:
    static inline volatile bool finishedTx{false};
    static inline volatile bool hasErrorTx{false};
};  // class Usart1Dma
}  // namespace platform
}  // namespace modm

#endif  // MODM_STM32_UART_1_DMA_HPP
