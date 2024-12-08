/*
 * HC05.h
 *
 *  Created on: Dec 7, 2024
 *      Author: markwilkinson
 */

#ifndef CUSTOMDRIVERS_BLE_HC05_H_
#define CUSTOMDRIVERS_BLE_HC05_H_

#include "stm32f3xx_hal.h"
#include <stdbool.h>

#define MAX_HC05_BUFFER_LEN 100u

bool hc05_init(UART_HandleTypeDef *huart);
void hc05_uart_send_tx(const char *buffer, uint16_t buffer_len);
void hc05_uart_handle_tx();
void hc05_uart_handle_rx();

#endif /* CUSTOMDRIVERS_BLE_HC05_H_ */
