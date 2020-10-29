// #include <modm/architecture/interface/interrupt.hpp>
// #include <modm/platform/clock/rcc.hpp>
// #include <modm/architecture/interface/register.hpp>

// int i = 0;

// #define SET_BIT(REG, BIT)     ((REG) |= (BIT))

// #define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

// #define READ_BIT(REG, BIT)    ((REG) & (BIT))

// #define CLEAR_REG(REG)        ((REG) = (0x0))

// #define WRITE_REG(REG, VAL)   ((REG) = (VAL))

// #define READ_REG(REG)         ((REG))

// #define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK)))
// | (SETMASK)))

// MODM_ISR(DMA2_Stream2)
// {
//     i++;
// 	/* clear idle it flag avoid idle interrupt all the time */

// 	/* handle received data in idle interrupt */
// // 	if (huart == &DBUS_HUART)
// // 	{
// // 		/* clear DMA transfer complete flag */
// // 		__HAL_DMA_DISABLE(huart->hdmarx);
// // // 50 - dma_counter == 18, start at 50
// // 		/* handle dbus data dbus_buf from DMA */
// // 		if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) ==
// DBUS_BUFLEN)
// // 		{
// // 			rc_callback_handler(&rc, dbus_buf);
// // 		}

// // 		/* restart dma transmission */
// // 		__HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);
// // 		__HAL_DMA_ENABLE(huart->hdmarx);
// // 	}
// }

// template<int T>
// struct Nvic;

// template <>
// struct Nvic<1>
// {
//     static constexpr IRQn_Type DmaIrqs[] {
//         DMA1_Stream1_IRQn,
//         DMA1_Stream2_IRQn,
//         DMA1_Stream3_IRQn,
//         DMA1_Stream4_IRQn,
//         DMA1_Stream5_IRQn,
//         DMA1_Stream6_IRQn,
//         DMA1_Stream7_IRQn,
//     };
// };

// template <>
// struct Nvic<2>
// {
//     static constexpr IRQn_Type DmaIrqs[] {
//         DMA2_Stream1_IRQn,
//         DMA2_Stream2_IRQn,
//         DMA2_Stream3_IRQn,
//         DMA2_Stream4_IRQn,
//         DMA2_Stream5_IRQn,
//         DMA2_Stream6_IRQn,
//         DMA2_Stream7_IRQn,
//     };
// };

// template<int ID>
// class Dma {
//     static_assert(ID == 1 || ID == 2, "invalid DMA controller ID");

// public:
// 	/**
// 	 * Enable the DMA controller in the RCC
// 	 */
// 	static void enable()
// 	{
// 		if constexpr (ID == 1)
// 			modm::platform::Rcc::enable<modm::platform::Peripheral::Dma1>();
// 		else
// 		    modm::platform::Rcc::enable<modm::platform::Peripheral::Dma2>();
// 	}
// 	/**
// 	 * Disable the DMA controller in the RCC
// 	 */
// 	static void
// 	disable()
// 	{
// 		if constexpr (ID == 1)
// 			modm::platform::Rcc::disable<modm::platform::Peripheral::Dma1>();
// 		else
// 			modm::platform::Rcc::disable<modm::platform::Peripheral::Dma2>();
// 	}

// 	enum class Interrupt {
// 		Global = 0x01,
// 		TransferComplete = 0x02,
// 		HalfTransferComplete = 0x04,
// 		Error = 0x08,
// 		All = 0x0f,
// 	};
// 	MODM_FLAGS32(Interrupt);

//     template<int StreamID>
//     class Stream {
//         /**
//          * Enable the IRQ vector of the channel
//          *
//          * @param[in] priority Priority of the IRQ
//          */
//         static void enableInterruptVector(uint32_t priority = 1)
//         {
//             NVIC_SetPriority(Nvic<ID>::DmaIrqs[StreamID], priority);
//             NVIC_EnableIRQ(Nvic<ID>::DmaIrqs[StreamID]);
//         }

//         /**
//          * Disable the IRQ vector of the channel
//          */
//         static void disableInterruptVector()
//         {
//             NVIC_DisableIRQ(Nvic<ID>::DmaIrqs[StreamID]);
//         }

//         /**
//          * Enable the specified interrupt of the channel
//          */
//         static void enableInterrupt(Interrupt_t irq)
//         {
//             ChannelHal::enableInterrupt(irq);
//         }

//         /**
//          * Disable the specified interrupt of the channel
//          */
//         static void disableInterrupt(Interrupt_t irq)
//         {
//             ChannelHal::disableInterrupt(irq);
//         }
//     };
// };

// void init() {
//   /* DMA controller clock enable */
//   /* DMA interrupt init */
//   /* DMA2_Stream2_IRQn interrupt configuration */

// //   open uart idle it

// }

// int dma_current_data_counter() {

// }

// int receive_dma() {

// }

// void rc_callback_handler() {

// }