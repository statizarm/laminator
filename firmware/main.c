#include "stm32f10x.h"
#include "ui.h"
#include "ili9341_driver.h"
#include "task.h"
#include "frame.h"
#include "global.h"
#include <string.h>
#include <stdio.h>
#include "application.h"

// Display controller - ili9341

// TODO: add menu

void delay(uint32_t);

void init(void);

int main(void)
{
	init_app();
	
	start_app();
}

void delay(uint32_t clocks)
{
	while(clocks > 0) {
		--clocks;
	}
}

void EXTI2_IRQHandler(void)
{
	EXTI->PR |= EXTI_PR_PR2;
	
	add_task(HANDLE_BTN_TASK_ID, 2);
}

void TIM4_IRQHandler()
{
	TIM4->SR &= ~TIM_SR_CC4IF;
	
	GPIOC->ODR ^= GPIO_ODR_ODR8;
		
	ADC1->CR2 |= ADC_CR2_SWSTART;
}

void ADC1_2_IRQHandler()
{
	ADC1->SR &= ~ADC_SR_EOC;
		
	add_task(FILTER_ADC_TASK_ID, 0);
}

void SysTick_Handler(void)
{
	// Частота возникающих прерываний - 24 KHz
	// Частота контроля режима нагрева - 12000Hz => anim_count_divisor = 24 KHz / 12 KHz = 2
	// Частота обновления температуры на дисплее - 10Hz => temp_count_divisor = 24KHz / 10 = 2400
	// Частота обновления анимации - 60Hz => anim_count_divisor = 24 KHz / 60 Hz = 400
	// Частота запросов на преобразование ADC - 24KHz
	// возможно что-то еще...
	static uint32_t count = 0;
	
	++count;
	
	if (count % 24000 == 0) {
		count = 0;
	}
	
	if (count % 2400 == 0) {
		add_task(REDRAW_ANIMATION_TASK_ID, 5);
	}
}

void EXTI15_10_IRQHandler(void)
{
	EXTI->PR |= EXTI_PR_PR13;
	// SLEEP BTN connected to pc13
	uint32_t *bit_band = (uint32_t *) (PERIPH_BB_BASE + ((uint32_t) &GPIOC->IDR - PERIPH_BASE) * 32 + 52);
	
	int count = 0;
	for (int i = 0; i < 256; ++i) {
		if (*bit_band) {
			++count;
		}
	}
	if (count < 128) {
		unsleep_app();
	} else {
		sleep_app();
	}
}

void TIM2_IRQHandler()
{
	TIM2->SR &= ~TIM_SR_UIF;
	
	add_task(HANDLE_ENC_TASK_ID, 3);
}
