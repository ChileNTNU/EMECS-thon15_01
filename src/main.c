/**************************************************************************//**
 * @file
 * @brief Simple LED Blink Demo for EFM32GG_STK3700
 * @version 3.20.12
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "bsp.h"
#include "bsp_trace.h"
#include "em_gpio.h"
#include "uart.h"
#include "i2c.h"

#define FRONT_VALUE  0xC4
#define BACK_VALUE   0x3A
#define FRONT_BACK_STEADY_VALUE  0xFA
#define LEFT_VALUE   0x40
#define RIGHT_VALUE  0xC2
#define LEFT_RIGHT_STEADY_VALUE  0x0A

volatile uint32_t msTicks; /* counts 1ms timeTicks */
volatile char rx_char = 0;

void Delay(uint32_t dlyTicks);

/**************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter
 *****************************************************************************/
void SysTick_Handler(void)
{
  msTicks++;       /* increment counter necessary in Delay()*/
}

/**************************************************************************//**
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 *****************************************************************************/
void Delay(uint32_t dlyTicks)
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) ;
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
	{
  CHIP_Init();                                   // This function addresses some chip errata and should be called at the start of every EFM32 application (need em_system.c)
  uint8_t i, front_back, left_right, command;
  char init_message[] = "Start!\n";

  /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) while (1) ;

  /* Initialize LED driver */
  BSP_LedsInit();
  BSP_LedSet(0);

  /* initalize clocks */
  CMU->CTRL |= (1 << 14);                         // Set HF clock divider to /2 to keep core frequency <32MHz
  CMU->OSCENCMD |= 0x4;                           // Enable XTAL Oscillator
  while(! (CMU->STATUS & 0x8) );                  // Wait for XTAL osc to stabilize
  CMU->CMD = 0x2;                                 // Select HF XTAL osc as system clock source. 48MHz XTAL, but we divided the system clock by 2, therefore our HF clock should be 24MHz

  usart_init();
  i2cInit();

  // Print test string
  for(i=0; init_message[i] != '\0'; i++) {
    usart_send_data(init_message[i]);
  }

  usart_enable_rx_isr();

  while (1)
  {
	performI2CTransfer();
    BSP_LedToggle(0);
    BSP_LedToggle(1);

    front_back = (uint8_t)(G[1]>>8 & 0xFF);
    left_right = (uint8_t)(G[0]>>8 & 0xFF);

/*
    usart_send_data(0xAA);
    usart_send_data(front_back);
    usart_send_data(0xBB);
    usart_send_data(left_right);
*/
    if (left_right >= 0x30 && front_back <= 0x45){
        	command = 'l';
        }else if (left_right >= 0xC0 && front_back <= 0xDF){
        	command = 'r';
        }else if(front_back >= 0xC0&& front_back <= 0xCD){
    	command = 'f';
    }else if (front_back >= 0x32 && front_back <= 0x35){
    	command = 'b';
    }

    /*else if (left_right >= 0x3E && front_back <= 0x40){
    	command = 'l';
    }else if (left_right >= 0xC4 && front_back <= 0xD0){
    	command = 'r';
    }
    */

    if(command != 0x00){
    	usart_send_data(command);
    	command = 0x00;
    }

    //usart_send_data((uint8_t)(G[0]>>8 & 0xFF));
    //usart_send_data((uint8_t)G[1] & 0xFF);

    Delay(500);
  }
}



