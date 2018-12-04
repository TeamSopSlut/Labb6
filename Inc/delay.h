/*
 * delay.h
 *
 *  Created on: 2 maj 2018
 *      Author: simon
 */

#ifndef DELAY_H_
#define DELAY_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

TIM_HandleTypeDef htim11;

void delay_us(uint32_t delay);

#endif /* DELAY_H_ */
