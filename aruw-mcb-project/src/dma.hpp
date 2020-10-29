#ifndef DMA_HPP_
#define DMA_HPP_

#include <stdint.h>
#include 
#include "dma_hal.hpp"

namespace aruwlib
{

namespace arch
{

/**
 * DMA controller
 *
 * Does not support - among other things - double buffering or FIFO usage
 *
 * @author	Mike Wolfram
 * @ingroup	modm_platform_dma
 */
template <uint32_t ID>
class DmaController : public DmaBase
{
	static_assert(ID == 1 || ID == 2);

public:
	/**
	 * Enable the DMA controller in the RCC
	 */
	static void
	enableRcc()
	{
		if constexpr (ID == 1)
        {
			Rcc::enable<Peripheral::Dma1>();
        }
		else
        {
			Rcc::enable<Peripheral::Dma2>();
        }
	}

	/**
	 * Disable the DMA controller in the RCC
	 */
	static void
	disableRcc()
	{
		if constexpr (ID == 1)
        {
			Rcc::disable<Peripheral::Dma1>();
        }
		else
        {
			Rcc::disable<Peripheral::Dma2>();
        }
	}

	/**
	 * Class representing a DMA channel/stream
	 */
	template <DmaBase::Stream StreamID>
	class Channel
	{
		using ControlHal = DmaHal<ID>;

		using STREAM_BASE = ControlHal::StreamBase() + uint32_t(StreamId) * ControlHal::CHANNEL_2_CHANNEL };

		using StreamHal = DmaStreamHal<StreamID, CHANNEL_BASE>;

	public:
		/**
		 * Configure the DMA channel
		 *
		 * Stops the DMA channel and writes the new values to its control register.
		 *
		 * @param[in] direction Direction of the DMA channel
		 * @param[in] memoryDataSize Size of data in memory (byte, halfword, word)
		 * @param[in] peripheralDataSize Size of data in peripheral (byte, halfword, word)
		 * @param[in] memoryIncrement Defines whether the memory address is incremented
		 * 			  after a transfer completed
		 * @param[in] peripheralIncrement Defines whether the peripheral address is
		 * 			  incremented after a transfer completed
		 * @param[in] priority Priority of the DMA channel
		 * @param[in] circularMode Transfer data in circular mode?
		 */
		static void
		configure(DataTransferDirection direction, MemoryDataSize memoryDataSize,
				PeripheralDataSize peripheralDataSize,
				MemoryIncrementMode memoryIncrement,
				PeripheralIncrementMode peripheralIncrement,
				Priority priority = Priority::Medium,
				CircularMode circularMode = CircularMode::Disabled)
		{
			ChannelHal::configure(direction, memoryDataSize, peripheralDataSize,
					memoryIncrement, peripheralIncrement, priority, circularMode);
		}

		/**
		 * Start the transfer of the DMA channel
		 */
		static void
		start()
		{
			ControlHal::clearInterruptFlags(uint32_t(Interrupt::Global) << (uint32_t(ChannelID) * 4));
			ChannelHal::start();
		}
		/**
		 * Stop a DMA channel transfer
		 */
		static void
		stop()
		{
			ChannelHal::stop();
		}

		/**
		 * Get the direction of the data transfer
		 */
		static DataTransferDirection
		getDataTransferDirection()
		{
			return ChannelHal::getDataTransferDirection();
		}

		/**
		 * Set the memory address of the DMA channel
		 *
		 * @note In Mem2Mem mode use this method to set the memory source address.
		 *
		 * @param[in] address Source address
		 */
		static void
		setMemoryAddress(uintptr_t address)
		{
			ChannelHal::setMemoryAddress(address);
		}
		/**
		 * Set the peripheral address of the DMA channel
		 *
		 * @note In Mem2Mem mode use this method to set the memory destination address.
		 *
		 * @param[in] address Destination address
		 */
		static void
		setPeripheralAddress(uintptr_t address)
		{
			ChannelHal::setPeripheralAddress(address);
		}

		/**
		 * Enable/disable memory increment
		 *
		 * When enabled, the memory address is incremented by the size of the data
		 * (e.g. 1 for byte transfers, 4 for word transfers) after the transfer
		 * completed.
		 *
		 * @param[in] increment Enable/disable
		 */
		static void
		setMemoryIncrementMode(bool increment)
		{
			ChannelHal::setMemoryIncrementMode(increment);
		}
		/**
		 * Enable/disable peripheral increment
		 *
		 * When enabled, the peripheral address is incremented by the size of the data
		 * (e.g. 1 for byte transfers, 4 for word transfers) after the transfer
		 * completed.
		 *
		 * @param[in] increment Enable/disable
		 */
		static void
		setPeripheralIncrementMode(bool increment)
		{
			ChannelHal::setPeripheralIncrementMode(increment);
		}

		/**
		 * Set the length of data to be transfered
		 */
		static void
		setDataLength(std::size_t length)
		{
			ChannelHal::setDataLength(length);
		}

		/**
		 * Set the IRQ handler for transfer errors
		 *
		 * The handler will be called from the channels IRQ handler function
		 * when the IRQ status indicates an error occured.
		 */
		static void
		setTransferErrorIrqHandler(IrqHandler irqHandler)
		{
			transferError = irqHandler;
		}
		/**
		 * Set the IRQ handler for transfer complete
		 *
		 * Called by the channels IRQ handler when the transfer is complete.
		 */
		static void
		setTransferCompleteIrqHandler(IrqHandler irqHandler)
		{
			transferComplete = irqHandler;
		}
		/**
		 * Set the peripheral that operates the channel
		 */
		template <DmaBase::Request dmaRequest>
		static void
		setPeripheralRequest()
		{
			DMA_Request_TypeDef *DMA_REQ = reinterpret_cast<DMA_Request_TypeDef *>(ControlHal::DMA_CSEL);
			DMA_REQ->CSELR &= ~(0x0f << (uint32_t(ChannelID) * 4));
			DMA_REQ->CSELR |= uint32_t(dmaRequest) << (uint32_t(ChannelID) * 4);
		}

		/**
		 * IRQ handler of the DMA channel
		 *
		 * Reads the IRQ status and checks for error or transfer complete. In case
		 * of error the DMA channel will be disabled.
		 */
		static void
		interruptHandler()
		{
			static const uint32_t TC_Flag {
				uint32_t(Interrupt::TransferComplete) << (uint32_t(ChannelID) * 4)
			};
			static const uint32_t TE_Flag {
				uint32_t(Interrupt::Error) << (uint32_t(ChannelID) * 4)
			};

			auto isr { ControlHal::getInterruptFlags() };
			if (isr & TE_Flag) {
				disable();
				if (transferError)
					transferError();
			}
			if ((isr & TC_Flag) and transferComplete)
				transferComplete();

			ControlHal::clearInterruptFlags(uint32_t(Interrupt::Global) << (uint32_t(ChannelID) * 4));
		}

		/**
		 * Enable the IRQ vector of the channel
		 *
		 * @param[in] priority Priority of the IRQ
		 */
		static void
		enableInterruptVector(uint32_t priority = 1)
		{
			NVIC_SetPriority(DmaBase::Nvic<ID>::DmaIrqs[uint32_t(ChannelID)], priority);
			NVIC_EnableIRQ(DmaBase::Nvic<ID>::DmaIrqs[uint32_t(ChannelID)]);
		}
		/**
		 * Disable the IRQ vector of the channel
		 */
		static void
		disableInterruptVector()
		{
			NVIC_DisableIRQ(DmaBase::Nvic<ID>::DmaIrqs[uint32_t(ChannelID)]);
		}

		/**
		 * Enable the specified interrupt of the channel
		 */
		static void
		enableInterrupt(Interrupt_t irq)
		{
			ChannelHal::enableInterrupt(irq);
		}
		/**
		 * Disable the specified interrupt of the channel
		 */
		static void
		disableInterrupt(Interrupt_t irq)
		{
			ChannelHal::disableInterrupt(irq);
		}

		/**
		 * Helper to verify that the selected channel supports the selected
		 * hardware and provides the Request to be set in setPeripheralRequest().
		 */
		template <Peripheral peripheral, Signal signal = Signal::NoSignal>
		struct RequestMapping {
		};

	private:
		static inline DmaBase::IrqHandler transferError { nullptr };
		static inline DmaBase::IrqHandler transferComplete { nullptr };
	};
};

/*
 * Derive DMA controller classes for convenience. Every derived class defines
 * the channels available on that controller.
 */
%% if target["family"] in ["f4"]
%% for streams in dma["streams"]
class Dma{{ streams.instance }}: public DmaController<{{ streams.instance }}>
{
public:
    %% for stream in streams.stream
    using Stream{{ stream.position }} = DmaController<{{ streams.instance }}>::Channel<DmaBase::Channel::Channel{{ stream.position }}>;
    %% endfor
};
%% endfor
%% elif target["family"] in ["f3", "l4"]
%% for channels in dma["channels"]
class Dma{{ channels.instance }}: public DmaController<{{ channels.instance }}>
{
public:
	%% for channel in channels.channel
	using Channel{{ channel.position }} = DmaController<{{ channels.instance }}>::Channel<DmaBase::Channel::Channel{{ channel.position }}>;
	%% endfor
};
%% endfor
%% endif


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
%% if target["family"] in ["f3", "l4"]
%% for channels in dma["channels"]
	%% for channel in channels.channel
		%% for request in channel.request
			%% for signal in request.signal
				%% set peripheral = signal.driver.capitalize()
				%% if signal.instance is defined
					%% set peripheral = peripheral ~ signal.instance
				%% else
					%% if peripheral not in ["Quadspi", "Aes", "Dcmi"]
						%% set peripheral = peripheral ~ 1
					%% endif
				%% endif
template <>
template <>
template <>
struct DmaController<{{ channels.instance }}>::Channel<DmaBase::Channel::Channel{{ channel.position }}>::RequestMapping<Peripheral::{{ peripheral }}{% if signal.name is defined %}, DmaBase::Signal::{{ signal.name.capitalize() }}{% endif %}>
{
	using Channel = DmaController<{{ channels.instance }}>::Channel<DmaBase::Channel::Channel{{ channel.position }}>;
	static constexpr DmaBase::Request Request = DmaBase::Request::Request{{ request.position }};
};

			%% endfor
		%% endfor
	%% endfor
%% endfor
%% endif
}	// namespace platform
}	// namespace modm


#endif  // DMA_HPP_

/**
 * DMA transaction:
 * sequence of a given number of data transfers
 * Three operations
 * - load from peripheral data register or a location in
 *   memory, addressed through DMA_SxPAR or DMA_SxM0AR register
 * - storage of data loaded to pheripheral data register or a location
 *   in memmory addressed through DMA_SxPAR or DMA_SxM0AR
 * - Post-decrement of DMA_SxNDTR register
 *
 * Channel selection:
 * Each stream has a DMA request, selected out of 8 possible channel requests
 * CHSEL[2:0] bits in DMA_SxCR register control selection
 * 8 requests come from peripherals (TIM, ADC, SPI, I2C, etc), and are independently
 * connected to each channel and connection depdnends on product implemtation
 *
 * Stream priority:
 * managed with the DMA_SxCR register, there are 4 priority levels
 *
 * Source, destination, and transfer modes
 * - direction configured using DIR[1:0] bits in DMA_SxCR register,
 *   different directions have differtent addresses associated with where
 *   to put data
 *  - 00: peripheral-to-memory
 *  - 01: memory-to-peripheral
 *  - 10: memory-to-memory
 *
 * Data width:
 * progrtammed in the PSIZE or MSIZE bits in the DMA_SxCR register
 *
 * Pointer incrementation:
 * PINC and MINC bits in DMA_SxCR register control if peripheral and memory
 * pointers are automatically post-incremented or kept constant after each
 * transfer
 * PINCOS bit in the DMA_SxCR register is used to align the increment offset
 * size for the peripheral address whatever the size of the data transferred
 * on the AHB peripheral port
 *
 * Circular mode
 * enable circualr mode using CIRC bit in DMA_SxCR register
 *
 *
 */