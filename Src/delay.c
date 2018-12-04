/*
 * delay.c
 *
 *  Created on: 2 maj 2018
 *      Author: simon
 */

#include "delay.h"

volatile uint32_t usTick = 0;

void delay_us(uint32_t delay)
{
	HAL_TIM_Base_Start_IT(&htim11);
	uint32_t current = usTick;
	while((usTick - current) < (delay - 2));
	HAL_TIM_Base_Stop_IT(&htim11);
}

