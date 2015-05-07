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

#define COM_PORT	gpioPortD //USART 1
#define TX_PIN		0
#define RX_PIN		1

volatile uint32_t msTicks; /* counts 1ms timeTicks */

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
  uint8_t i;
  char rx_char = 0;
  char test_string[] = "Start!\n";

  /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) while (1) ;

  /* Initialize LED driver */
  BSP_LedsInit();
  BSP_LedSet(0);

  CMU->CTRL |= (1 << 14);                         // Set HF clock divider to /2 to keep core frequency <32MHz
  CMU->OSCENCMD |= 0x4;                           // Enable XTAL Oscillator
  while(! (CMU->STATUS & 0x8) );                  // Wait for XTAL osc to stabilize
  CMU->CMD = 0x2;                                 // Select HF XTAL osc as system clock source. 48MHz XTAL, but we divided the system clock by 2, therefore our HF clock should be 24MHz

  CMU->HFPERCLKEN0 = (1 << 13) | (1 << 1);        // Enable GPIO, and USART1 peripheral clocks

  GPIO->P[COM_PORT].MODEL = (1 << 4) | (4 << 0);  // Configure PD0 as digital output and PD1 as input
  GPIO->P[COM_PORT].DOUTSET = (1 << TX_PIN); // Initialize PD0 high since UART TX idles high (otherwise glitches can occur)

  // Use default value for USART1->CTRL: asynch mode, x16 OVS, lsb first, CLK idle low
  // Default frame options: 8-none-1-none
  USART1->CLKDIV = (152 << 6);                               // 152 will give 38400 baud rate (using 16-bit oversampling with 24MHz peripheral clock)
  USART1->CMD = (1 << 11) | (1 << 10) | (1 << 2) | (1 << 0); // Clear RX/TX buffers and shif regs, Enable Transmitter and Receiver
  USART1->IFC = 0x1FF9;                                      // clear all USART interrupt flags
  USART1->ROUTE = 0x103;                                     // Enable TX and RX pins, use location #1 (UART TX and RX located at PD0 and PD1, see EFM32GG990 datasheet for details)

  // Print test string
  for(i=0; i<strlen(test_string); i++) {
    while( !(USART1->STATUS & (1 << 6)) ); // wait for TX buffer to empty
    USART1->TXDATA = test_string[i];
  }

#if 0
  while (1)
  {
    BSP_LedToggle(0);
    BSP_LedToggle(1);

    USART_Tx( USART1, data);

    Delay(1000);
  }
#endif

#if 1
  while(1) {
    if(USART1->STATUS & (1 << 7)) {   // if RX buffer contains valid data
      rx_char = USART1->RXDATA;       // store the data
    }
    if(rx_char) {                     // if we have a valid character
      if(USART1->STATUS & (1 << 6)) { // check if TX buffer is empty
        USART1->TXDATA = rx_char;     // echo received char
        rx_char = 0;                  // reset temp variable
      }
    }
  }
#endif

#if 0
  uint8_t data = 0xAA;

  /* Chip errata */
  CHIP_Init();

  /* If first word of user data page is non-zero, enable eA Profiler trace */
  BSP_TraceProfilerSetup();

  /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) while (1) ;

  /* Initialize LED driver */
  BSP_LedsInit();
  BSP_LedSet(0);


  CMU_ClockDivSet(cmuClock_HF, cmuClkDiv_2);
  CMU_OscillatorEnable(cmuOsc_HFXO, true, true);
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

  /* Init USART */
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_USART1, true);
  GPIO_PinModeSet(COM_PORT, TX_PIN, gpioModePushPull, 1);
  GPIO_PinModeSet(COM_PORT, RX_PIN, gpioModeInput, 0);

  USART_InitAsync_TypeDef usartInit = USART_INITASYNC_DEFAULT;
  USART_InitAsync( USART1, &usartInit );
  USART1->ROUTE = UART_ROUTE_RXPEN | UART_ROUTE_TXPEN | _UART_ROUTE_LOCATION_LOC1;

  USART_Enable( USART1, usartEnable);

  /* Infinite blink loop */
  while (1)
  {
    BSP_LedToggle(0);
    BSP_LedToggle(1);

    USART_Tx( USART1, data);

    Delay(1000);
  }
#endif
}
