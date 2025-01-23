/*
 * Copyright (c) 2021, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once

#include <stdint.h>
#include <modm/architecture/utils.hpp>
#include <string_view>

extern "C"
{

void Reset_Handler(void);
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void WWDG_IRQHandler(void);
void PVD_IRQHandler(void);
void TAMP_STAMP_IRQHandler(void);
void RTC_WKUP_IRQHandler(void);
void FLASH_IRQHandler(void);
void RCC_IRQHandler(void);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void DMA1_Stream0_IRQHandler(void);
void DMA1_Stream1_IRQHandler(void);
void DMA1_Stream2_IRQHandler(void);
void DMA1_Stream3_IRQHandler(void);
void DMA1_Stream4_IRQHandler(void);
void DMA1_Stream5_IRQHandler(void);
void DMA1_Stream6_IRQHandler(void);
void ADC_IRQHandler(void);
void CAN1_TX_IRQHandler(void);
void CAN1_RX0_IRQHandler(void);
void CAN1_RX1_IRQHandler(void);
void CAN1_SCE_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void TIM1_BRK_TIM9_IRQHandler(void);
void TIM1_UP_TIM10_IRQHandler(void);
void TIM1_TRG_COM_TIM11_IRQHandler(void);
void TIM1_CC_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
void I2C2_EV_IRQHandler(void);
void I2C2_ER_IRQHandler(void);
void SPI1_IRQHandler(void);
void SPI2_IRQHandler(void);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void RTC_Alarm_IRQHandler(void);
void OTG_FS_WKUP_IRQHandler(void);
void TIM8_BRK_TIM12_IRQHandler(void);
void TIM8_UP_TIM13_IRQHandler(void);
void TIM8_TRG_COM_TIM14_IRQHandler(void);
void TIM8_CC_IRQHandler(void);
void DMA1_Stream7_IRQHandler(void);
void FMC_IRQHandler(void);
void SDIO_IRQHandler(void);
void TIM5_IRQHandler(void);
void SPI3_IRQHandler(void);
void UART4_IRQHandler(void);
void UART5_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);
void TIM7_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);
void DMA2_Stream1_IRQHandler(void);
void DMA2_Stream2_IRQHandler(void);
void DMA2_Stream3_IRQHandler(void);
void DMA2_Stream4_IRQHandler(void);
void ETH_IRQHandler(void);
void ETH_WKUP_IRQHandler(void);
void CAN2_TX_IRQHandler(void);
void CAN2_RX0_IRQHandler(void);
void CAN2_RX1_IRQHandler(void);
void CAN2_SCE_IRQHandler(void);
void OTG_FS_IRQHandler(void);
void DMA2_Stream5_IRQHandler(void);
void DMA2_Stream6_IRQHandler(void);
void DMA2_Stream7_IRQHandler(void);
void USART6_IRQHandler(void);
void I2C3_EV_IRQHandler(void);
void I2C3_ER_IRQHandler(void);
void OTG_HS_EP1_OUT_IRQHandler(void);
void OTG_HS_EP1_IN_IRQHandler(void);
void OTG_HS_WKUP_IRQHandler(void);
void OTG_HS_IRQHandler(void);
void DCMI_IRQHandler(void);
void HASH_RNG_IRQHandler(void);
void FPU_IRQHandler(void);
void UART7_IRQHandler(void);
void UART8_IRQHandler(void);
void SPI4_IRQHandler(void);
void SPI5_IRQHandler(void);
void SPI6_IRQHandler(void);
void SAI1_IRQHandler(void);
void DMA2D_IRQHandler(void);
}

namespace modm::platform::detail
{

constexpr std::string_view vectorNames[] =
{
	"__main_stack_top",
	"Reset",
	"NMI",
	"HardFault",
	"MemManage",
	"BusFault",
	"UsageFault",
	"Undefined",
	"Undefined",
	"Undefined",
	"Undefined",
	"SVC",
	"DebugMon",
	"Undefined",
	"PendSV",
	"SysTick",
	"WWDG",
	"PVD",
	"TAMP_STAMP",
	"RTC_WKUP",
	"FLASH",
	"RCC",
	"EXTI0",
	"EXTI1",
	"EXTI2",
	"EXTI3",
	"EXTI4",
	"DMA1_Stream0",
	"DMA1_Stream1",
	"DMA1_Stream2",
	"DMA1_Stream3",
	"DMA1_Stream4",
	"DMA1_Stream5",
	"DMA1_Stream6",
	"ADC",
	"CAN1_TX",
	"CAN1_RX0",
	"CAN1_RX1",
	"CAN1_SCE",
	"EXTI9_5",
	"TIM1_BRK_TIM9",
	"TIM1_UP_TIM10",
	"TIM1_TRG_COM_TIM11",
	"TIM1_CC",
	"TIM2",
	"TIM3",
	"TIM4",
	"I2C1_EV",
	"I2C1_ER",
	"I2C2_EV",
	"I2C2_ER",
	"SPI1",
	"SPI2",
	"USART1",
	"USART2",
	"USART3",
	"EXTI15_10",
	"RTC_Alarm",
	"OTG_FS_WKUP",
	"TIM8_BRK_TIM12",
	"TIM8_UP_TIM13",
	"TIM8_TRG_COM_TIM14",
	"TIM8_CC",
	"DMA1_Stream7",
	"FMC",
	"SDIO",
	"TIM5",
	"SPI3",
	"UART4",
	"UART5",
	"TIM6_DAC",
	"TIM7",
	"DMA2_Stream0",
	"DMA2_Stream1",
	"DMA2_Stream2",
	"DMA2_Stream3",
	"DMA2_Stream4",
	"ETH",
	"ETH_WKUP",
	"CAN2_TX",
	"CAN2_RX0",
	"CAN2_RX1",
	"CAN2_SCE",
	"OTG_FS",
	"DMA2_Stream5",
	"DMA2_Stream6",
	"DMA2_Stream7",
	"USART6",
	"I2C3_EV",
	"I2C3_ER",
	"OTG_HS_EP1_OUT",
	"OTG_HS_EP1_IN",
	"OTG_HS_WKUP",
	"OTG_HS",
	"DCMI",
	"Undefined",
	"HASH_RNG",
	"FPU",
	"UART7",
	"UART8",
	"SPI4",
	"SPI5",
	"SPI6",
	"SAI1",
	"Undefined",
	"Undefined",
	"DMA2D",
};


#ifndef MODM_ISR_DISABLE_VALIDATION
#define MODM_ISR_VALIDATE(vector_str, vector) \
	static_assert(::modm::platform::detail::validateIrqName(vector_str), \
			"'" vector_str "' is not a valid IRQ name!\n" \
			"  Hint: You do not need to add '_IRQHandler' to the name.\n" \
			"  Hint: Here are all the IRQs on this device:\n" \
			"    - WWDG\n" \
			"    - PVD\n" \
			"    - TAMP_STAMP\n" \
			"    - RTC_WKUP\n" \
			"    - FLASH\n" \
			"    - RCC\n" \
			"    - EXTI0\n" \
			"    - EXTI1\n" \
			"    - EXTI2\n" \
			"    - EXTI3\n" \
			"    - EXTI4\n" \
			"    - DMA1_Stream0\n" \
			"    - DMA1_Stream1\n" \
			"    - DMA1_Stream2\n" \
			"    - DMA1_Stream3\n" \
			"    - DMA1_Stream4\n" \
			"    - DMA1_Stream5\n" \
			"    - DMA1_Stream6\n" \
			"    - ADC\n" \
			"    - CAN1_TX\n" \
			"    - CAN1_RX0\n" \
			"    - CAN1_RX1\n" \
			"    - CAN1_SCE\n" \
			"    - EXTI9_5\n" \
			"    - TIM1_BRK_TIM9\n" \
			"    - TIM1_UP_TIM10\n" \
			"    - TIM1_TRG_COM_TIM11\n" \
			"    - TIM1_CC\n" \
			"    - TIM2\n" \
			"    - TIM3\n" \
			"    - TIM4\n" \
			"    - I2C1_EV\n" \
			"    - I2C1_ER\n" \
			"    - I2C2_EV\n" \
			"    - I2C2_ER\n" \
			"    - SPI1\n" \
			"    - SPI2\n" \
			"    - USART1\n" \
			"    - USART2\n" \
			"    - USART3\n" \
			"    - EXTI15_10\n" \
			"    - RTC_Alarm\n" \
			"    - OTG_FS_WKUP\n" \
			"    - TIM8_BRK_TIM12\n" \
			"    - TIM8_UP_TIM13\n" \
			"    - TIM8_TRG_COM_TIM14\n" \
			"    - TIM8_CC\n" \
			"    - DMA1_Stream7\n" \
			"    - FMC\n" \
			"    - SDIO\n" \
			"    - TIM5\n" \
			"    - SPI3\n" \
			"    - UART4\n" \
			"    - UART5\n" \
			"    - TIM6_DAC\n" \
			"    - TIM7\n" \
			"    - DMA2_Stream0\n" \
			"    - DMA2_Stream1\n" \
			"    - DMA2_Stream2\n" \
			"    - DMA2_Stream3\n" \
			"    - DMA2_Stream4\n" \
			"    - ETH\n" \
			"    - ETH_WKUP\n" \
			"    - CAN2_TX\n" \
			"    - CAN2_RX0\n" \
			"    - CAN2_RX1\n" \
			"    - CAN2_SCE\n" \
			"    - OTG_FS\n" \
			"    - DMA2_Stream5\n" \
			"    - DMA2_Stream6\n" \
			"    - DMA2_Stream7\n" \
			"    - USART6\n" \
			"    - I2C3_EV\n" \
			"    - I2C3_ER\n" \
			"    - OTG_HS_EP1_OUT\n" \
			"    - OTG_HS_EP1_IN\n" \
			"    - OTG_HS_WKUP\n" \
			"    - OTG_HS\n" \
			"    - DCMI\n" \
			"    - HASH_RNG\n" \
			"    - FPU\n" \
			"    - UART7\n" \
			"    - UART8\n" \
			"    - SPI4\n" \
			"    - SPI5\n" \
			"    - SPI6\n" \
			"    - SAI1\n" \
			"    - DMA2D\n" \
	)
#else
#define MODM_ISR_VALIDATE(...)
#endif

constexpr int getIrqPosition(std::string_view name)
{
	for (int pos = 0; pos < 107; pos++)
		if (vectorNames[pos] == name) return pos;
	return -1;
}

constexpr bool validateIrqName(std::string_view name)
{
	return getIrqPosition(name) != -1;
}

}	// namespace modm::platform::detail