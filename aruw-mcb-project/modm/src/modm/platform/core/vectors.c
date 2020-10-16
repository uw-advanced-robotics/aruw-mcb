/*
 * Copyright (c) 2018, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include <stdint.h>
#include <modm/architecture/utils.hpp>
#include <modm/architecture/interface/assert.h>

// ----------------------------------------------------------------------------
extern void modm_undefined_handler(int32_t);
void Undefined_Handler(void)
{
	int32_t irqn;
	asm volatile("mrs %[irqn], ipsr" :[irqn] "=r" (irqn));
	modm_undefined_handler(irqn - 16);
}
/* Provide weak aliases for each Exception handler to Undefined_Handler.
 * As they are weak aliases, any function with the same name will override
 * this definition. */
void Reset_Handler(void)						__attribute__((noreturn));
void NMI_Handler(void)							__attribute__((weak, alias("Undefined_Handler")));
void HardFault_Handler(void)					__attribute__((weak, alias("Undefined_Handler")));
void SVC_Handler(void)							__attribute__((weak, alias("Undefined_Handler")));
void PendSV_Handler(void)						__attribute__((weak, alias("Undefined_Handler")));
void SysTick_Handler(void)						__attribute__((weak, alias("Undefined_Handler")));
void WWDG_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void PVD_VDDIO2_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void RTC_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void FLASH_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void RCC_CRS_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void EXTI0_1_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void EXTI2_3_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void EXTI4_15_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void TSC_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void DMA1_Channel1_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void DMA1_Channel2_3_IRQHandler(void)			__attribute__((weak, alias("Undefined_Handler")));
void DMA1_Channel4_5_6_7_IRQHandler(void)		__attribute__((weak, alias("Undefined_Handler")));
void ADC1_COMP_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)		__attribute__((weak, alias("Undefined_Handler")));
void TIM1_CC_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void TIM2_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void TIM3_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void TIM6_DAC_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void TIM7_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void TIM14_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void TIM15_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void TIM16_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void TIM17_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void I2C1_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void I2C2_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void SPI1_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void SPI2_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void USART1_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void USART2_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void USART3_4_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void CEC_CAN_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void USB_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
// ----------------------------------------------------------------------------
typedef void (* const FunctionPointer)(void);

// defined in the linkerscript
extern uint32_t __main_stack_top[];
extern uint32_t __process_stack_top[];

// Define the vector table
modm_section(".vector_rom")
FunctionPointer vectorsRom[] =
{
	(FunctionPointer)__main_stack_top,		// -16: stack pointer
	Reset_Handler,							// -15: code entry point
	NMI_Handler,							// -14: Non Maskable Interrupt handler
	HardFault_Handler,						// -13: hard fault handler
	Undefined_Handler,						// -12
	Undefined_Handler,						// -11
	Undefined_Handler,						// -10
	Undefined_Handler,						//  -9
	Undefined_Handler,						//  -8
	Undefined_Handler,						//  -7
	Undefined_Handler,						//  -6
	SVC_Handler,							//  -5
	Undefined_Handler,						//  -4
	Undefined_Handler,						//  -3
	PendSV_Handler,							//  -2
	SysTick_Handler,						//  -1
	WWDG_IRQHandler,						//   0
	PVD_VDDIO2_IRQHandler,					//   1
	RTC_IRQHandler,							//   2
	FLASH_IRQHandler,						//   3
	RCC_CRS_IRQHandler,						//   4
	EXTI0_1_IRQHandler,						//   5
	EXTI2_3_IRQHandler,						//   6
	EXTI4_15_IRQHandler,					//   7
	TSC_IRQHandler,							//   8
	DMA1_Channel1_IRQHandler,				//   9
	DMA1_Channel2_3_IRQHandler,				//  10
	DMA1_Channel4_5_6_7_IRQHandler,			//  11
	ADC1_COMP_IRQHandler,					//  12
	TIM1_BRK_UP_TRG_COM_IRQHandler,			//  13
	TIM1_CC_IRQHandler,						//  14
	TIM2_IRQHandler,						//  15
	TIM3_IRQHandler,						//  16
	TIM6_DAC_IRQHandler,					//  17
	TIM7_IRQHandler,						//  18
	TIM14_IRQHandler,						//  19
	TIM15_IRQHandler,						//  20
	TIM16_IRQHandler,						//  21
	TIM17_IRQHandler,						//  22
	I2C1_IRQHandler,						//  23
	I2C2_IRQHandler,						//  24
	SPI1_IRQHandler,						//  25
	SPI2_IRQHandler,						//  26
	USART1_IRQHandler,						//  27
	USART2_IRQHandler,						//  28
	USART3_4_IRQHandler,					//  29
	CEC_CAN_IRQHandler,						//  30
	USB_IRQHandler,							//  31
};
