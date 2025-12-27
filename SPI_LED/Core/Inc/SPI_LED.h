/*
 * SPI_LED.h
 *
 *  Created on: Dec 24, 2025
 *      Author: admin
 */

#ifndef INC_SPI_LED_H_
#define INC_SPI_LED_H_

#include "main.h"
#include "Config.h"

uint8_t spi_led_buffer[BUFFER_SIZE];

void spi_update(void)
{
	extern SPI_HandleTypeDef hspi3;
    // Start the DMA transfer
    HAL_SPI_Transmit_DMA(&hspi3, spi_led_buffer, BUFFER_SIZE);
}

void spi_set_led(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
    if (index < 0 || index >= NUM_LEDS) {
        return; // Out of bounds
    }

    // Calculate the starting position for this LED in the buffer
    // (skip the 4-byte start frame)
    uint32_t pos = START_FRAME_SIZE + (index * LED_FRAME_SIZE);

    // APA102 LED Frame:
    // Byte 0: 0xE0 (Global Brightness, 5 bits)
    // Byte 1: Blue (0-255)
    // Byte 2: Green (0-255)
    // Byte 3: Red (0-255)

    spi_led_buffer[pos + 0] = 0xE0 | (brightness & 0x1F); // Brightness (masked to 5 bits)
    spi_led_buffer[pos + 1] = b;
    spi_led_buffer[pos + 2] = g;
    spi_led_buffer[pos + 3] = r;
}

// Hàm này tự chuẩn bị buffer cho dây SPI
void spi_init_buffer(void) {
	for (int i = 0; i < START_FRAME_SIZE; i++) spi_led_buffer[i] = 0x00;
	int end_pos = START_FRAME_SIZE + (NUM_LEDS * LED_FRAME_SIZE);
	for (int i = 0; i < END_FRAME_SIZE; i++) spi_led_buffer[end_pos + i] = 0xFF;

    // Tắt đèn
    for(int i=0; i<NUM_LEDS; i++) spi_set_led(i, 0,0,0,0);
}

#endif /* INC_SPI_LED_H_ */
