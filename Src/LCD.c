/*
 * LCD.c
 *
 *  Created on: 2 maj 2018
 *      Author: Simon
 */

#include "LCD.h"
#include "delay.h"

extern uint32_t usTick;


void TextLCD_Strobe(TextLCDType *lcd);
void TextLCD_Cmd(TextLCDType *lcd, uint8_t cmd);
void TextLCD_Data(TextLCDType *lcd, uint8_t data);



void TextLCD_Strobe(TextLCDType *lcd) // Enables the E-pin to write things
{
	delay_us(40);
	HAL_GPIO_WritePin(lcd->controlPort, lcd->strbPin, 1);
	delay_us(40);
	HAL_GPIO_WritePin(lcd->controlPort, lcd->strbPin, 0);
	delay_us(40);
}
void TextLCD_Cmd(TextLCDType *lcd, uint8_t cmd) // Runs your command(bit-pattern)
{
	HAL_GPIO_WritePin(lcd->controlPort, lcd->rsPin | lcd->rwPin, 0);
	lcd->dataPort->ODR &=~ 0xff;
	lcd->dataPort->ODR |= cmd;

	TextLCD_Strobe(lcd);
}
void TextLCD_Data(TextLCDType *lcd, uint8_t data) // Sends a bit-pattern on the databus.
{
	HAL_GPIO_WritePin(lcd->controlPort, lcd->rsPin, 1);
	HAL_GPIO_WritePin(lcd->controlPort, lcd->rwPin, 0);
	lcd->dataPort->ODR &= ~0xff;
	lcd->dataPort->ODR |= data;

	TextLCD_Strobe(lcd);
}

void TextLCD_Init(TextLCDType *lcd, GPIO_TypeDef *controlPort, uint16_t rsPin, uint16_t rwPin, uint16_t enPin, GPIO_TypeDef *dataPort, uint16_t dataPins)
{
	lcd->controlPort = controlPort;
	lcd->rsPin = rsPin;
	lcd->rwPin = rwPin;
	lcd->strbPin = enPin;
	lcd->dataPort = dataPort;
	lcd->dataPins = dataPins;

	HAL_Delay(20);
	TextLCD_Cmd(lcd, 0x38);
	HAL_Delay(6);
	TextLCD_Cmd(lcd, 0x38);
	delay_us(100);
	TextLCD_Cmd(lcd, 0x38);
	delay_us(40);
	TextLCD_Cmd(lcd, 0x38);
	delay_us(40);
	TextLCD_Cmd(lcd, 0x06);
	delay_us(40);
	TextLCD_Cmd(lcd, 0x0E);
	delay_us(40);
	TextLCD_Cmd(lcd, 0x01);
	HAL_Delay(3);
	TextLCD_Cmd(lcd, 0x80);
}
void TextLCD_Home (TextLCDType *lcd) // cursor jumps back to the first position
{
	TextLCD_Cmd(lcd, 0x02);
	HAL_Delay(2);
}
void TextLCD_Clear (TextLCDType *lcd) // clears display
{
	TextLCD_Cmd(lcd, 0x01);
	HAL_Delay(2);
}
void TextLCD_Position (TextLCDType *lcd, int x, int y)
{
	TextLCD_Cmd(lcd, 0x01 * x | 0x80 | 0x40 * y);
	delay_us(40);
}
void TextLCD_Putchar (TextLCDType *lcd, uint8_t data)
{
	TextLCD_Data(lcd, data);
	delay_us(40);
}
void TextLCD_PutInt (TextLCDType *lcd, uint32_t data){
	if(data < 0){
		TextLCD_Putchar(lcd, '-');
		data = data * -1;
	}
	if(data > 9)
		TextLCD_PutInt(lcd, data/10);

	TextLCD_Putchar(lcd, data % 10 + 0x30);
}
void TextLCD_Puts (TextLCDType *lcd, char *string)
{
	uint32_t counter = 0;
	while(*string != '\0'){
		TextLCD_Putchar(lcd, *string);
		string++;
		counter++;
		if(counter > 15){
			TextLCD_Position(lcd, 0, 1);
			counter = 0;
		}
	}
}
void TextLCD_Printf (TextLCDType *lcd, char *message, ...)
{

}



