/*
 * periph_uart.c
 *
 *  Created on: Oct 6, 2024
 *      Author: markwilkinson
 */


#include "periph_uart.h"
#include <string.h>

/***** PRIVATE VARIABLES *****/
// Used for Interrupt UART exercise
static UART_HandleTypeDef huart_s;
static bool uart_init_success;
static uint8_t tx_buffer_s[MAX_BUFFER_LEN]; // tx_data buffer
static uint8_t rx_buffer_s[MAX_BUFFER_LEN]; // rx_data buffer
static uint8_t rx_data_s; // receive buffer
static uint32_t counter_s = 0; // count how many bytes are received

const char* hello_cmd = "hello";

/***** PUBLIC FUNCTIONS *****/
/*!
 * \brief     Setup function to init uart with the correct data buffers
 * \param[in] huart - pointer to the uart handle we want to use.
 * \return    init_okay - True if setup was okay, false if not.
 */
bool periph_uart_init(UART_HandleTypeDef *huart)
{
	huart_s = *huart;

	// Here these functions need called to get the UART ready to receive and transmit data over the UART using interrupts.
	// Data sent out over the UART must be formatted, added to the tx_buffer_s buffer and sent from this buffer.
	// Data received into UART is stored in rx_data_s buffer 1 character at a time which allows the user to look for specific characters.
	HAL_StatusTypeDef tx_setup_ok = HAL_UART_Transmit_IT(huart, tx_buffer_s, MAX_BUFFER_LEN);
	HAL_StatusTypeDef rx_setup_ok = HAL_UART_Receive_IT(huart, &rx_data_s, 1);

	uart_init_success = (tx_setup_ok == HAL_OK && rx_setup_ok == HAL_OK);

	return uart_init_success;
}

/*!
 * \brief    Sends content in the buffer over uart tx
 * \param[in] tx_buff - Buffer with the message we want to send.
 * \param[in] buffer_len - Maximum length of the buffer we want to send.
 */
void periph_uart_send_tx(const char *buffer, uint16_t buffer_len)
{
	if (!uart_init_success)
	{
		return;
	}

	strcpy((char *)tx_buffer_s, buffer);
	HAL_UART_Transmit(&huart_s, tx_buffer_s, buffer_len, 100);
}

/*!
 * \brief    Readies the uart tx buffer
 * \param[in] huart - pointer to huart handle.
 */
void periph_uart_handle_tx()
{
	if (!uart_init_success)
	{
		return;
	}

	memset(tx_buffer_s, '\0', (size_t)MAX_BUFFER_LEN); //empty the transmit data buffer to be ready for new data.
}

/*!
 * \brief    Handles uart rx by taking in next character, processing it, then sending out over tx
 * \param[in] huart - pointer to huart handle.
 */
void periph_uart_handle_rx()
{
	if (!uart_init_success)
	{
		return;
	}

	// if we didn't receive the carriage return increment the rx_buffer and add the next character
	if (rx_data_s != '\r')
	{
		// if there is data coming into the rx_data pointer that isn't the "Enter" character then add it to the buffer.
		rx_buffer_s[counter_s++] = rx_data_s;
		// Get ready for new data in rx_data_s pointer.
		HAL_UART_Receive_IT(&huart_s, &rx_data_s, 1);
		return;
	}

	// otherwise we did get a carriage return, check against our expected string
	if (strcmp(hello_cmd, (char*)rx_buffer_s) == 0) // string compare the command against what we received from the sender.
	{
		const char *response = "Hello to you too!\r\n";
		strcpy((char*)tx_buffer_s, response); // move response into tx buffer
	}
	else
	{
		// otherwise put an error in the tx buffer.
		const char *error = "Uh oh, didn't get the right word...\r\n";
		strcpy((char*)tx_buffer_s, error);
	}

	// now transmit out the answer over tx.
	HAL_UART_Transmit(&huart_s, tx_buffer_s, (uint16_t)MAX_BUFFER_LEN, 100);
	// clear both buffers
	memset(tx_buffer_s, '\0', (size_t)MAX_BUFFER_LEN);
	memset(rx_buffer_s, '\0', (size_t)MAX_BUFFER_LEN);
	counter_s = 0; // reset the counter to be ready for new data.

	// Get ready for new data in rx_data_s pointer.
	HAL_UART_Receive_IT(&huart_s, &rx_data_s, 1);
}
