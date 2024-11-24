/*
 * periph_uart.h
 *
 *  Created on: Oct 6, 2024
 *      Author: markwilkinson
 */

#ifndef PERIPHS_PERIPH_UART_H_
#define PERIPHS_PERIPH_UART_H_

#include "stm32f3xx_hal.h"
#include <stdbool.h>


#define MAX_BUFFER_LEN 100u

bool periph_uart_init(UART_HandleTypeDef *huart);
void periph_uart_send_tx(const char *buffer, uint16_t buffer_len);
void periph_uart_handle_tx(UART_HandleTypeDef *huart);
void periph_uart_handle_rx(UART_HandleTypeDef *huart);

#endif /* PERIPHS_PERIPH_UART_H_ */
