/*
 * UART_LED.h
 *
 *  Created on: Dec 24, 2025
 *      Author: admin
 */

#ifndef INC_UART_LED_H_
#define INC_UART_LED_H_

#include "main.h"
#include "Config.h"

extern USART_HandleTypeDef husart6;

uint8_t usart_led_buffer[BUFFER_SIZE] = {0};

void usart_set_led(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);
void usart_update(void);
void usart_init_buffer(void);

void usart_set_led(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
    if (index >= NUM_LEDS) {
        return;
    }

    uint32_t pos = START_FRAME_SIZE + (index * LED_FRAME_SIZE);

    usart_led_buffer[pos + 0] = 0xE0 | (brightness & 0x1F);
    usart_led_buffer[pos + 1] = b;
    usart_led_buffer[pos + 2] = g;
    usart_led_buffer[pos + 3] = r;
}

void usart_update(void)
{
    // Wait for any previous DMA transfer to finish
    // Note: We check HAL_UART_STATE_BUSY_TX
    while (HAL_USART_GetState(&husart6) == HAL_USART_STATE_BUSY_TX);

    // Start the DMA transfer using the UART HAL function
    HAL_USART_Transmit_DMA(&husart6, usart_led_buffer, BUFFER_SIZE);
}

void usart_init_buffer(void) {
	for (int i = 0; i < START_FRAME_SIZE; i++) usart_led_buffer[i] = 0x00;
	int end_pos = START_FRAME_SIZE + (NUM_LEDS * LED_FRAME_SIZE);
	for (int i = 0; i < END_FRAME_SIZE; i++) usart_led_buffer[end_pos + i] = 0xFF;

    for(int i=0; i<NUM_LEDS; i++) usart_set_led(i, 0,0,0,0);
}

#endif /* INC_UART_LED_H_ */
