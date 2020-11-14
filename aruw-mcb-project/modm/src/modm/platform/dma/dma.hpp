/*
 * Copyright (c) 2014, Kevin Läufer
 * Copyright (c) 2014-2017, Niklas Hauser
 * Copyright (c) 2020, Mike Wolfram
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_DMA_HPP
#define MODM_STM32_DMA_HPP

#include <stdint.h>

#include <modm/platform/clock/rcc.hpp>

#include "dma_hal.hpp"

namespace modm
{
namespace platform
{
/**
 * DMA controller for stm32f4 series microcontrollers
 */
template <uint32_t ID>
class DmaController : public DmaBase
{
    static_assert(ID >= 1 && ID <= 2, "dma id invalid");

public:
    /**
     * Enable the DMA controller in the RCC.
     */
    static void enableRcc()
    {
        if constexpr (ID == 1)
        {
            Rcc::enable<Peripheral::Dma1>();
        }
        else if constexpr (ID == 2)
        {
            Rcc::enable<Peripheral::Dma2>();
        }
    }

    /**
     * Disable the DMA controller in the RCC.
     */
    static void disableRcc()
    {
        if constexpr (ID == 1)
        {
            Rcc::disable<Peripheral::Dma1>();
        }
        else if constexpr (ID == 2)
        {
            Rcc::disable<Peripheral::Dma2>();
        }
    }

    /**
     * Class representing a DMA channel/stream.
     */
    template <DmaBase::StreamID SID>
    class Stream
    {
        using ControlHal = DmaHal<ID>;
        using StreamHal = DmaStreamHal<SID, ControlHal::streamBase()>;

    public:
        static constexpr uint32_t DMA_ID = ID;
        static constexpr DmaBase::StreamID STREAM_ID = SID;

        /**
         * Configure the DMA channel
         *
         * @see DmaStreamHal::configure
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
            StreamHal::configure(
                channel,
                direction,
                periphInc,
                memInc,
                peripDataSize,
                memDataSize,
                mode,
                priority,
                fifoMode,
                fifoThreshold,
                memBurst,
                periphBurst);
        }

        static void selectChannel(DmaBase::ChannelSelection channel)
        {
            StreamHal::selectChannel(channel);
        }

        /**
         * Start the transfer of the DMA channel
         */
        static void enable()
        {
            ControlHal::clearInterruptFlags(STREAM_ID);
            StreamHal::enable();
        }
        /**
         * Stop a DMA channel transfer
         */
        static bool disable() { return StreamHal::disable(); }

        /**
         * Get the direction of the data transfer
         */
        static DataTransferDirection getDataTransferDirection()
        {
            return StreamHal::getDataTransferDirection();
        }

        /**
         * Configure the DMA channel to write to the requested data buffer length bytes.
         *
         * @note When in continuous read mode, other read operations are disabled.
         */
        static bool configureRead(uint8_t *data, std::size_t length)
        {
            // Disable to allow modifications of the DMA stream control register.
            if (!disable())
            {
                return false;
            }
            // configure total # of bytes
            setDataLength(length);
            // Write the memory address in the DMA control register to configure it as the
            // destination of the transfer. The data will be loaded from USART_DR to this memory
            // area after each RXNE event.
            setDestinationAddress(reinterpret_cast<uintptr_t>(data));
            // activate channel in DMA control register
            enable();
            return true;
        }

        static bool configureWrite(uint8_t *data, std::size_t length)
        {
            if (disable())
            {
                return false;
            }
            setDataLength(length);
            setSourceAddress(reinterpret_cast<uintptr_t>(data));
            enable();
            return true;
        }

        /**
         * Set the memory address of the DMA channel
         *
         * @note In Mem2Mem mode use this method to set the memory source address.
         *
         * @param[in] address Source address
         */
        static void setSourceAddress(uintptr_t address) { StreamHal::setSourceAddress(address); }

        /**
         * Set the peripheral address of the DMA channel
         *
         * @note In Mem2Mem mode use this method to set the memory destination address.
         *
         * @param[in] address Destination address
         */
        static void setDestinationAddress(uintptr_t address)
        {
            StreamHal::setDestinationAddress(address);
        }

        /**
         * Set the length of data to be transfered
         */
        static void setDataLength(std::size_t length) { StreamHal::setDataLength(length); }

        /**
         * Set the IRQ handler for transfer errors.
         *
         * The handler will be called from the channels IRQ handler function
         * when the IRQ status indicates an error occured.
         */
        static void setTransferErrorIrqHandler(IrqHandler irqHandler)
        {
            transferError = irqHandler;
        }

        /**
         * Set the IRQ handler for transfer complete.
         *
         * Called by the channels IRQ handler when the transfer is complete.
         */
        static void setTransferCompleteIrqHandler(IrqHandler irqHandler)
        {
            transferComplete = irqHandler;
        }

        static uint32_t getInterruptFlags() { return ControlHal::getInterruptFlags(STREAM_ID); }

        /**
         * IRQ handler of the DMA channel.
         *
         * Reads the IRQ status and checks for error or transfer complete. In case
         * of error the DMA channel will be disabled.
         */
        static void interruptHandler()
        {
            uint32_t currIsrs = ControlHal::getInterruptFlags(STREAM_ID);
            ControlHal::clearInterruptFlags(STREAM_ID);
            if (currIsrs & (uint32_t(InterruptStatusMsks::TRANSFER_ERROR) |
                            uint32_t(InterruptStatusMsks::DIRECT_MODE_ERROR) |
                            uint32_t(InterruptStatusMsks::FIFO_ERROR)))
            {
                if (transferError != nullptr)
                {
                    transferError();
                }
            }
            if ((currIsrs & uint32_t(InterruptStatusMsks::TRANSFER_COMPLETE)) && transferComplete)
            {
                transferComplete();
            }
        }

        /**
         * Enable the IRQ vector of the channel.
         *
         * @param[in] priority Priority of the IRQ
         */
        static void enableInterruptVector(uint32_t priority = 1)
        {
            NVIC_SetPriority(DmaBase::Nvic<ID>::DmaIrqs[uint32_t(STREAM_ID)], priority);
            NVIC_EnableIRQ(DmaBase::Nvic<ID>::DmaIrqs[uint32_t(STREAM_ID)]);
        }

        /**
         * Disable the IRQ vector of the channel.
         */
        static void disableInterruptVector()
        {
            NVIC_DisableIRQ(DmaBase::Nvic<ID>::DmaIrqs[uint32_t(STREAM_ID)]);
        }

        /**
         * Enable the specified interrupt of the channel.
         */
        static void enableInterrupt(Interrupt_t irq) { StreamHal::enableInterrupt(irq); }

        /**
         * Disable the specified interrupt of the channel.
         */
        static void disableInterrupt(Interrupt_t irq) { StreamHal::disableInterrupt(irq); }

        template<DmaBase::ChannelSelection Ch, Peripheral Periph, Signal Sig>
        struct RequestMapping {
        };
    private:
        static inline DmaBase::IrqHandler transferError{nullptr};
        static inline DmaBase::IrqHandler transferComplete{nullptr};
    };
};  // class DmaController

/*
 * Derive DMA controller classes for convenience. Every derived class defines
 * the streams available on that controller.
 */
class Dma1 : public DmaController<1>
{
public:
    using Stream0 = DmaController<1>::Stream<DmaBase::StreamID::STREAM_0>;
    using Stream1 = DmaController<1>::Stream<DmaBase::StreamID::STREAM_1>;
    using Stream2 = DmaController<1>::Stream<DmaBase::StreamID::STREAM_2>;
    using Stream3 = DmaController<1>::Stream<DmaBase::StreamID::STREAM_3>;
    using Stream4 = DmaController<1>::Stream<DmaBase::StreamID::STREAM_4>;
    using Stream5 = DmaController<1>::Stream<DmaBase::StreamID::STREAM_5>;
    using Stream6 = DmaController<1>::Stream<DmaBase::StreamID::STREAM_6>;
    using Stream7 = DmaController<1>::Stream<DmaBase::StreamID::STREAM_7>;
};  // class Dma1

class Dma2 : public DmaController<2>
{
public:
    using Stream0 = DmaController<2>::Stream<DmaBase::StreamID::STREAM_0>;
    using Stream1 = DmaController<2>::Stream<DmaBase::StreamID::STREAM_1>;
    using Stream2 = DmaController<2>::Stream<DmaBase::StreamID::STREAM_2>;
    using Stream3 = DmaController<2>::Stream<DmaBase::StreamID::STREAM_3>;
    using Stream4 = DmaController<2>::Stream<DmaBase::StreamID::STREAM_4>;
    using Stream5 = DmaController<2>::Stream<DmaBase::StreamID::STREAM_5>;
    using Stream6 = DmaController<2>::Stream<DmaBase::StreamID::STREAM_6>;
    using Stream7 = DmaController<2>::Stream<DmaBase::StreamID::STREAM_7>;
};  // class Dma1

/*
 * Specialization of the RequestMapping. For all hardware supported by DMA the
 * RequestMapping structure defines the channel and the Request. It can be used
 * by hardware classes to verify that the provided channel is valid and to
 * get the value to set in setPeripheralRequest().
 *
 * Example:
 * template <class DmaRx, class DmaTx>
 * class SpiMaster1_Dma : public SpiMaster1
 * {
 *     using RxChannel = typename DmaRx::template RequestMapping<Peripheral::Spi1, DmaBase::Signal::Rx>::Channel;
 * 	   using TxChannel = typename DmaTx::template RequestMapping<Peripheral::Spi1, DmaBase::Signal::Tx>::Channel;
 * 	   static constexpr DmaBase::Request RxRequest = DmaRx::template RequestMapping<Peripheral::Spi1, DmaBase::Signal::Rx>::Request;
 * 	   static constexpr DmaBase::Request TxRequest = DmaTx::template RequestMapping<Peripheral::Spi1, DmaBase::Signal::Tx>::Request;
 *
 *     ...
 * };
 */
template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_0>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_0, Peripheral::Spi3, DmaBase::Signal::Rx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_0>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_0;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_0>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_1, Peripheral::I2c1, DmaBase::Signal::Rx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_0>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_1;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_0>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_2, Peripheral::Tim4, DmaBase::Signal::Ch1>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_0>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_2;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_0>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_3, Peripheral::I2s3, DmaBase::Signal::Ext_rx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_0>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_3;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_0>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_4, Peripheral::Uart5, DmaBase::Signal::Rx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_0>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_4;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_0>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_5, Peripheral::Uart8, DmaBase::Signal::Tx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_0>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_5;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_0>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_6, Peripheral::Tim5, DmaBase::Signal::Ch3>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_0>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_6;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_0>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_6, Peripheral::Tim5, DmaBase::Signal::Up>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_0>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_6;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_1>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_3, Peripheral::Tim2, DmaBase::Signal::Ch3>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_1>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_3;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_1>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_3, Peripheral::Tim2, DmaBase::Signal::Up>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_1>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_3;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_1>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_4, Peripheral::Usart3, DmaBase::Signal::Rx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_1>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_4;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_1>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_5, Peripheral::Uart7, DmaBase::Signal::Tx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_1>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_5;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_1>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_6, Peripheral::Tim5, DmaBase::Signal::Ch4>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_1>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_6;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_1>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_6, Peripheral::Tim5, DmaBase::Signal::Trig>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_1>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_6;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_1>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_7, Peripheral::Tim6, DmaBase::Signal::Up>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_1>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_7;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_2>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_0, Peripheral::Spi3, DmaBase::Signal::Rx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_2>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_0;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_2>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_1, Peripheral::Tim7, DmaBase::Signal::Up>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_2>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_1;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_2>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_2, Peripheral::I2s3, DmaBase::Signal::Ext_rx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_2>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_2;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_2>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_3, Peripheral::I2c3, DmaBase::Signal::Rx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_2>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_3;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_2>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_4, Peripheral::Uart4, DmaBase::Signal::Rx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_2>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_4;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_2>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_5, Peripheral::Tim3, DmaBase::Signal::Ch4>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_2>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_5;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_2>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_5, Peripheral::Tim3, DmaBase::Signal::Up>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_2>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_5;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_2>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_6, Peripheral::Tim5, DmaBase::Signal::Ch1>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_2>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_6;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_2>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_7, Peripheral::I2c2, DmaBase::Signal::Rx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_2>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_7;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_3>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_0, Peripheral::Spi2, DmaBase::Signal::Rx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_3>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_0;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_3>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_2, Peripheral::Tim4, DmaBase::Signal::Ch2>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_3>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_2;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_3>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_3, Peripheral::I2s2, DmaBase::Signal::Ext_rx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_3>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_3;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_3>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_4, Peripheral::Usart3, DmaBase::Signal::Tx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_3>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_4;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_3>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_5, Peripheral::Uart7, DmaBase::Signal::Rx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_3>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_5;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_3>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_6, Peripheral::Tim5, DmaBase::Signal::Ch4>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_3>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_6;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_3>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_6, Peripheral::Tim5, DmaBase::Signal::Trig>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_3>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_6;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_3>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_7, Peripheral::I2c2, DmaBase::Signal::Rx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_3>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_7;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_4>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_0, Peripheral::Spi2, DmaBase::Signal::Tx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_4>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_0;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_4>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_1, Peripheral::Tim7, DmaBase::Signal::Up>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_4>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_1;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_4>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_2, Peripheral::I2s2, DmaBase::Signal::Ext_tx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_4>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_2;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_4>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_3, Peripheral::I2c3, DmaBase::Signal::Tx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_4>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_3;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_4>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_4, Peripheral::Uart4, DmaBase::Signal::Tx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_4>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_4;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_4>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_5, Peripheral::Tim3, DmaBase::Signal::Ch1>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_4>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_5;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_4>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_5, Peripheral::Tim3, DmaBase::Signal::Trig>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_4>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_5;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_4>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_6, Peripheral::Tim5, DmaBase::Signal::Ch2>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_4>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_6;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_4>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_7, Peripheral::Usart3, DmaBase::Signal::Tx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_4>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_7;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_5>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_0, Peripheral::Spi3, DmaBase::Signal::Tx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_5>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_0;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_5>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_1, Peripheral::I2c1, DmaBase::Signal::Rx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_5>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_1;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_5>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_2, Peripheral::I2s3, DmaBase::Signal::Ext_tx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_5>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_2;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_5>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_3, Peripheral::Tim2, DmaBase::Signal::Ch1>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_5>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_3;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_5>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_4, Peripheral::Usart2, DmaBase::Signal::Rx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_5>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_4;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_5>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_5, Peripheral::Tim3, DmaBase::Signal::Ch2>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_5>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_5;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_5>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_7, Peripheral::Dac, DmaBase::Signal::Dac1>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_5>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_7;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_6>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_1, Peripheral::I2c1, DmaBase::Signal::Tx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_6>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_1;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_6>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_2, Peripheral::Tim4, DmaBase::Signal::Up>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_6>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_2;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_6>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_3, Peripheral::Tim2, DmaBase::Signal::Ch2>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_6>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_3;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_6>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_3, Peripheral::Tim2, DmaBase::Signal::Ch4>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_6>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_3;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_6>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_4, Peripheral::Usart2, DmaBase::Signal::Tx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_6>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_4;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_6>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_5, Peripheral::Uart8, DmaBase::Signal::Rx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_6>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_5;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_6>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_6, Peripheral::Tim5, DmaBase::Signal::Up>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_6>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_6;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_6>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_7, Peripheral::Dac, DmaBase::Signal::Dac2>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_6>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_7;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_7>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_0, Peripheral::Spi3, DmaBase::Signal::Tx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_7>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_0;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_7>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_1, Peripheral::I2c1, DmaBase::Signal::Tx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_7>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_1;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_7>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_2, Peripheral::Tim4, DmaBase::Signal::Ch3>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_7>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_2;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_7>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_3, Peripheral::Tim2, DmaBase::Signal::Ch4>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_7>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_3;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_7>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_3, Peripheral::Tim2, DmaBase::Signal::Up>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_7>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_3;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_7>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_4, Peripheral::Uart5, DmaBase::Signal::Tx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_7>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_4;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_7>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_5, Peripheral::Tim3, DmaBase::Signal::Ch3>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_7>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_5;
};

template<>
template<>
template<>
struct DmaController<1>::Stream<DmaBase::StreamID::STREAM_7>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_7, Peripheral::I2c2, DmaBase::Signal::Tx>
{
    using Stream = DmaController<1>::Stream<DmaBase::StreamID::STREAM_7>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_7;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_0>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_0, Peripheral::Adc1, DmaBase::Signal::NoSignal>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_0>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_0;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_0>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_2, Peripheral::Adc3, DmaBase::Signal::NoSignal>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_0>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_2;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_0>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_3, Peripheral::Spi1, DmaBase::Signal::Rx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_0>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_3;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_0>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_4, Peripheral::Spi4, DmaBase::Signal::Rx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_0>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_4;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_0>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_6, Peripheral::Tim1, DmaBase::Signal::Trig>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_0>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_6;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_1>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_0, Peripheral::Sai1, DmaBase::Signal::A>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_1>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_0;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_1>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_1, Peripheral::Dcmi, DmaBase::Signal::NoSignal>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_1>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_1;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_1>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_2, Peripheral::Adc3, DmaBase::Signal::NoSignal>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_1>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_2;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_1>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_4, Peripheral::Spi4, DmaBase::Signal::Tx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_1>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_4;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_1>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_5, Peripheral::Usart6, DmaBase::Signal::Rx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_1>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_5;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_1>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_6, Peripheral::Tim1, DmaBase::Signal::Ch1>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_1>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_6;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_1>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_7, Peripheral::Tim8, DmaBase::Signal::Up>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_1>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_7;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_2>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_0, Peripheral::Tim8, DmaBase::Signal::Ch1>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_2>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_0;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_2>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_0, Peripheral::Tim8, DmaBase::Signal::Ch2>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_2>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_0;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_2>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_0, Peripheral::Tim8, DmaBase::Signal::Ch3>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_2>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_0;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_2>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_1, Peripheral::Adc2, DmaBase::Signal::NoSignal>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_2>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_1;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_2>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_3, Peripheral::Spi1, DmaBase::Signal::Rx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_2>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_3;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_2>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_4, Peripheral::Usart1, DmaBase::Signal::Rx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_2>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_4;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_2>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_5, Peripheral::Usart6, DmaBase::Signal::Rx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_2>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_5;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_2>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_6, Peripheral::Tim1, DmaBase::Signal::Ch2>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_2>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_6;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_2>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_7, Peripheral::Tim8, DmaBase::Signal::Ch1>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_2>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_7;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_3>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_0, Peripheral::Sai1, DmaBase::Signal::A>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_3>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_0;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_3>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_1, Peripheral::Adc2, DmaBase::Signal::NoSignal>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_3>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_1;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_3>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_2, Peripheral::Spi5, DmaBase::Signal::Rx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_3>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_2;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_3>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_3, Peripheral::Spi1, DmaBase::Signal::Tx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_3>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_3;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_3>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_4, Peripheral::Sdio, DmaBase::Signal::Rx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_3>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_4;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_3>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_4, Peripheral::Sdio, DmaBase::Signal::Tx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_3>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_4;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_3>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_5, Peripheral::Spi4, DmaBase::Signal::Rx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_3>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_5;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_3>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_6, Peripheral::Tim1, DmaBase::Signal::Ch1>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_3>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_6;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_3>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_7, Peripheral::Tim8, DmaBase::Signal::Ch2>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_3>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_7;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_4>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_0, Peripheral::Adc1, DmaBase::Signal::NoSignal>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_4>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_0;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_4>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_1, Peripheral::Sai1, DmaBase::Signal::B>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_4>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_1;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_4>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_2, Peripheral::Spi5, DmaBase::Signal::Tx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_4>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_2;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_4>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_5, Peripheral::Spi4, DmaBase::Signal::Tx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_4>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_5;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_4>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_6, Peripheral::Tim1, DmaBase::Signal::Ch4>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_4>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_6;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_4>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_6, Peripheral::Tim1, DmaBase::Signal::Com>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_4>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_6;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_4>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_6, Peripheral::Tim1, DmaBase::Signal::Trig>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_4>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_6;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_4>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_7, Peripheral::Tim8, DmaBase::Signal::Ch3>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_4>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_7;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_5>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_0, Peripheral::Sai1, DmaBase::Signal::B>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_5>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_0;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_5>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_1, Peripheral::Spi6, DmaBase::Signal::Tx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_5>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_1;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_5>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_3, Peripheral::Spi1, DmaBase::Signal::Tx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_5>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_3;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_5>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_4, Peripheral::Usart1, DmaBase::Signal::Rx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_5>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_4;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_5>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_6, Peripheral::Tim1, DmaBase::Signal::Up>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_5>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_6;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_5>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_7, Peripheral::Spi5, DmaBase::Signal::Rx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_5>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_7;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_6>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_0, Peripheral::Tim1, DmaBase::Signal::Ch1>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_6>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_0;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_6>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_0, Peripheral::Tim1, DmaBase::Signal::Ch2>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_6>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_0;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_6>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_0, Peripheral::Tim1, DmaBase::Signal::Ch3>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_6>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_0;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_6>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_1, Peripheral::Spi6, DmaBase::Signal::Rx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_6>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_1;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_6>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_4, Peripheral::Sdio, DmaBase::Signal::Rx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_6>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_4;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_6>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_4, Peripheral::Sdio, DmaBase::Signal::Tx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_6>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_4;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_6>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_5, Peripheral::Usart6, DmaBase::Signal::Tx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_6>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_5;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_6>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_6, Peripheral::Tim1, DmaBase::Signal::Ch3>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_6>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_6;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_6>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_7, Peripheral::Spi5, DmaBase::Signal::Tx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_6>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_7;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_7>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_1, Peripheral::Dcmi, DmaBase::Signal::NoSignal>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_7>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_1;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_7>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_4, Peripheral::Usart1, DmaBase::Signal::Tx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_7>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_4;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_7>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_5, Peripheral::Usart6, DmaBase::Signal::Tx>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_7>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_5;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_7>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_7, Peripheral::Tim8, DmaBase::Signal::Ch4>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_7>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_7;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_7>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_7, Peripheral::Tim8, DmaBase::Signal::Com>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_7>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_7;
};

template<>
template<>
template<>
struct DmaController<2>::Stream<DmaBase::StreamID::STREAM_7>::RequestMapping<DmaBase::ChannelSelection::CHANNEL_7, Peripheral::Tim8, DmaBase::Signal::Trig>
{
    using Stream = DmaController<2>::Stream<DmaBase::StreamID::STREAM_7>;
    static constexpr DmaBase::ChannelSelection Channel = DmaBase::ChannelSelection::CHANNEL_7;
};

}  // namespace platform
}  // namespace modm

#endif  // MODM_STM32_DMA_HPP
