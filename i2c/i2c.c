/*
 * i2c.c
 *
 *  Created on: Feb 25, 2017
 *      Author: Eason
 */


#include "i2c.h"

#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
//#include "misc.h" //for nvic stuff
//#include <stdio.h>

void init_I2C1(void){

	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;

	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//already initialized in usart1.c
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	//Tell pins PB8 and PB9 which alternating function you will use
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1); //tx
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1); //rx

	//setup GPIO pins 8 and 9 for alternate functions
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	//setup i2c1 for 100khz, 7 bit addressing, etc.
	I2C_InitStruct.I2C_ClockSpeed = 100000;
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_OwnAddress1 = 0x0; //master
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;

	//enable I2C
	I2C_Init(I2C1, &I2C_InitStruct);
	I2C_Cmd(I2C1,ENABLE);
}
//todo: send and receive functions etc. add interrupt/handler?
