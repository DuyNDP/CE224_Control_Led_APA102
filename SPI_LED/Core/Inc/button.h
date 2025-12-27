/*
 * button.h
 * Quản lý nút bấm PE0 (SPI) và PE1 (USART)
 * Logic: Active High (Nhấn = 1, Nhả = 0)
 * Yêu cầu GPIO: Input Pull-Down
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include "main.h"

// --- CẤU HÌNH ---
#define BTN_SPI_PIN     GPIO_PIN_0
#define BTN_SPI_PORT    GPIOE
#define BTN_UART_PIN    GPIO_PIN_1
#define BTN_UART_PORT   GPIOE
#define BTN_RESET_PIN   GPIO_PIN_8
#define BTN_RESET_PORT  GPIOB

#define DEBOUNCE_DELAY  50      // 50ms chống rung
#define MAX_EFFECTS     8       // Tổng số hiệu ứng bạn có (sửa số này theo thực tế)

// --- BIẾN TRẠNG THÁI HIỆU ỨNG (Toàn cục) ---
// Các file khác (main.c, effect.h) sẽ dùng biến này để biết đang chạy hiệu ứng gì
volatile uint8_t effect_mode_spi  = 0; // Mode cho dải SPI
volatile uint8_t effect_mode_uart = 0; // Mode cho dải USART

// --- BIẾN NỘI BỘ CHO XỬ LÝ NÚT ---
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    uint8_t last_state;
    uint32_t last_time;
    uint8_t is_pressed;
} Button_t;

Button_t btn_spi =  {BTN_SPI_PORT,  BTN_SPI_PIN,  0, 0, 0};
Button_t btn_uart = {BTN_UART_PORT, BTN_UART_PIN, 0, 0, 0};
Button_t btn_reset = {BTN_RESET_PORT, BTN_RESET_PIN, 0, 0, 0};

// --- HÀM XỬ LÝ LOGIC NÚT BẤM ---
// Hàm này cần được gọi liên tục trong while(1) của main.c
void button_scan(void) {
    uint32_t current_time = HAL_GetTick();

    // 1. XỬ LÝ NÚT SPI (PE0)
    uint8_t read_spi = HAL_GPIO_ReadPin(btn_spi.port, btn_spi.pin);

    // Nếu trạng thái thay đổi, reset timer (đang rung)
    if (read_spi != btn_spi.last_state) {
        btn_spi.last_time = current_time;
    }

    // Nếu trạng thái ổn định trong khoảng DEBOUNCE_DELAY
    if ((current_time - btn_spi.last_time) > DEBOUNCE_DELAY) {
        // Nếu đọc thấy mức 1 (Đang nhấn) và trước đó chưa xác nhận nhấn
        if (read_spi == 1 && btn_spi.is_pressed == 0) {
            btn_spi.is_pressed = 1; // Đánh dấu đã nhấn

            // --> THAY ĐỔI HIỆU ỨNG SPI
            effect_mode_spi++;
            if (effect_mode_spi >= MAX_EFFECTS) effect_mode_spi = 0;
        }

        // Nếu nút đã nhả (về 0)
        if (read_spi == 0) {
            btn_spi.is_pressed = 0; // Reset trạng thái để chờ lần nhấn sau
        }
    }
    btn_spi.last_state = read_spi;


    // 2. XỬ LÝ NÚT USART (PE1) - Logic tương tự
    uint8_t read_uart = HAL_GPIO_ReadPin(btn_uart.port, btn_uart.pin);

    if (read_uart != btn_uart.last_state) {
        btn_uart.last_time = current_time;
    }

    if ((current_time - btn_uart.last_time) > DEBOUNCE_DELAY) {
        if (read_uart == 1 && btn_uart.is_pressed == 0) {
            btn_uart.is_pressed = 1;

            // --> THAY ĐỔI HIỆU ỨNG USART
            effect_mode_uart++;
            if (effect_mode_uart >= MAX_EFFECTS) effect_mode_uart = 0;
        }

        if (read_uart == 0) {
            btn_uart.is_pressed = 0;
        }
    }
    btn_uart.last_state = read_uart;


    // 3. XỬ LÝ NÚT RESET
    uint8_t read_rst = HAL_GPIO_ReadPin(btn_reset.port, btn_reset.pin);

    // Khử rung cho nút Reset
    if (read_rst != btn_reset.last_state) btn_reset.last_time = current_time;

    if ((current_time - btn_reset.last_time) > DEBOUNCE_DELAY) {
    	// Chỉ kích hoạt khi nhấn xuống (Active High)
        if (read_rst == 1 && btn_reset.is_pressed == 0) {
        	btn_reset.is_pressed = 1;

            // ==> GỌI HÀM RESET TỪ EFFECT.H <==
            reset_system_effects();
        }

        if (read_rst == 0) {
        	btn_reset.is_pressed = 0;
        }
    }
    btn_reset.last_state = read_rst;
}

#endif /* INC_BUTTON_H_ */
