/*
 * uart.h
 *
 *  Created on: 07.05.2015
 *      Author: Alex
 */

#ifndef UART_H_
#define UART_H_

#include <stdint.h>

extern uint8_t rx_data;

void usart_enable_rx_isr(void);
void usart_send_data(uint8_t data);
void usart_init(void);

#endif /* UART_H_ */
