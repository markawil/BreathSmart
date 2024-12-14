/*
 * HC05.c
 *
 *  Created on: Dec 7, 2024
 *      Author: markwilkinson
 */

#include "HC05.h"
#include "periph_uart.h"
#include "stm32f3xx_hal.h"
#include <string.h>

bool get_address();
bool check_ok();

static UART_HandleTypeDef huart_s;
static uint8_t tx_buffer_s[MAX_HC05_BUFFER_LEN]; // tx_data buffer
static uint8_t rx_buffer_s[MAX_HC05_BUFFER_LEN]; // rx_data buffer
static uint8_t rx_data; // receive buffer
static uint32_t counter_s = 0; // count how many bytes are received
const char hc05_error[] = "Error! Something failed for HC05 module\r\n";

bool hc05_init(UART_HandleTypeDef *huart)
{
	huart_s = *huart;

	// Here these functions must be called to get the UART ready to receive and transmit data over the UART using interrupts.
	// Data sent out over the UART must be formatted, added to the tx_buffer_s buffer and sent from this buffer.
	// Data received into UART is stored in rx_data_s buffer 1 character at a time which allows the user to look for specific characters.
	HAL_StatusTypeDef rx_setup_ok = HAL_UART_Receive_IT(huart, &rx_data, 1);
	HAL_StatusTypeDef tx_setup_ok = HAL_UART_Transmit_IT(huart, tx_buffer_s, MAX_HC05_BUFFER_LEN);

	if (tx_setup_ok != HAL_OK || rx_setup_ok != HAL_OK)
	{
		return false;
	}

	// both will send responses to the serial out UART for debugging,
	if (!check_ok())
	{
		return false;
	}

	if (!check_ok())
	{
		return false;
	}

//	if (!get_address())
//	{
//		return false;
//	}

	return true;
}

bool check_ok()
{
	char buffer[2] = "AT";
	bool result = hc05_uart_send_tx(buffer, sizeof(buffer));
	return result;
}

bool get_address()
{
	char buffer[8] = "AT+ADDR?";
	bool result = hc05_uart_send_tx(buffer, sizeof(buffer));
	return result;
}

/*!
 * \brief    Sends content in the buffer over uart tx to the HC-05 module
 * \param[in] buffer - Buffer with the message we want to send.
 * \param[in] buffer_len - Maximum length of the buffer we want to send.
 */
bool hc05_uart_send_tx(const char *buffer, uint16_t buffer_len)
{
	strcpy((char *)tx_buffer_s, buffer);
	HAL_StatusTypeDef result = HAL_UART_Transmit(&huart_s, tx_buffer_s, buffer_len, 100);
	if (result != HAL_OK)
	{
		periph_uart_send_tx(hc05_error, sizeof(hc05_error));
		return false;
	}

	return true;
}

/*!
 * \brief    Readies the uart tx buffer
 * \param[in] huart - pointer to huart handle.
 */
void hc05_uart_handle_tx()
{
	memset(tx_buffer_s, '\0', (size_t)MAX_BUFFER_LEN); //empty the transmit data buffer to be ready for new data.
}

/*
 * \brief Handles the incoming data from HC05 module
 */
void hc05_uart_handle_rx()
{
	// store the next character
	//rx_buffer_s[counter_s++] = rx_data;

	// send the received character out over the serial output TX
	periph_uart_send_tx((char*)&rx_data, 1);

//	// if we've received a character already and previous character was O and current is K
//	if (counter_s > 1 && rx_buffer_s[counter_s - 1] == 'O' && rx_data_s == 'K')
//	{
//		// HC05 responded with OK, we can clear the buffers, send carriage return
//		const char *endline = "\r\n";
//		periph_uart_send_tx(endline, sizeof(endline));
//		memset(tx_buffer_s, '\0', (size_t)MAX_BUFFER_LEN);
//		memset(rx_buffer_s, '\0', (size_t)MAX_BUFFER_LEN);
//		counter_s = 0; // reset the counter to be ready for next line of data.
//	}
//	else
//	{
		// Get ready for new data in rx_data_s pointer.
		HAL_UART_Receive_IT(&huart_s, &rx_data, 1);
//	}
}

