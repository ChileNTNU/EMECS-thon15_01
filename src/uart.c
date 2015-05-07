/*
 * uart.c
 *
 *  Created on: 07.05.2015
 *      Author: Alex
 */

#include "uart.h"
#include "efm32gg990f1024.h"
#include "em_usart.h"
#include "em_gpio.h"

uint8_t rx_data;

#define COM_PORT	gpioPortD //USART 1
#define TX_PIN		0
#define RX_PIN		1

void usart_init(void) {
	CMU->HFPERCLKEN0 = (1 << 13) | (1 << 1);        // Enable GPIO, and USART1 peripheral clocks

	GPIO->P[COM_PORT].MODEL = (1 << 4) | (4 << 0);  // Configure PD0 as digital output and PD1 as input
	GPIO->P[COM_PORT].DOUTSET = (1 << TX_PIN); // Initialize PD0 high since UART TX idles high (otherwise glitches can occur)

	// Use default value for USART1->CTRL: asynch mode, x16 OVS, lsb first, CLK idle low
	// Default frame options: 8-none-1-none
	USART1->CLKDIV = (152 << 6);                               // 152 will give 38400 baud rate (using 16-bit oversampling with 24MHz peripheral clock)
	USART1->CMD = (1 << 11) | (1 << 10) | (1 << 2) | (1 << 0); // Clear RX/TX buffers and shif regs, Enable Transmitter and Receiver
	USART1->IFC = 0x1FF9;                                      // clear all USART interrupt flags
	USART1->ROUTE = 0x103;                                     // Enable TX and RX pins, use location #1 (UART TX and RX located at PD0 and PD1, see EFM32GG990 datasheet for details)

}

void usart_enable_rx_isr(void) {
	USART1->IEN = USART_IEN_RXDATAV;
	NVIC_EnableIRQ(USART1_RX_IRQn);
}

void usart_send_data(uint8_t data) {
	USART_Tx( USART1, data);
}

void USART1_RX_IRQHandler(void) {
	if(USART1->STATUS & (1 << 7)) {   // if RX buffer contains valid data
	      rx_data = USART1->RXDATA;       // store the data
	    }
	    if(rx_data) {                     // if we have a valid character
	      if(USART1->STATUS & (1 << 6)) { // check if TX buffer is empty
	        USART1->TXDATA = rx_data;     // echo received char
	      }
	    }
}
