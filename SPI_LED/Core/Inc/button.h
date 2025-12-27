/*
 * button.h
 * LOGIC:
 * - PE0: Đổi hiệu ứng dây SPI (0 -> 8 -> 0)
 * - PE1: Đổi hiệu ứng dây USART (0 -> 6 -> 0)
 * - PB8: Reset (Tắt 3s -> Về Breathing)
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include "main.h"
#include "effect.h" // Import thư viện effect để gọi hàm clear và update

// --- CẤU HÌNH PIN ---
#define BTN_SPI_PIN     GPIO_PIN_0
#define BTN_SPI_PORT    GPIOE
#define BTN_UART_PIN    GPIO_PIN_1
#define BTN_UART_PORT   GPIOE
#define BTN_RESET_PIN   GPIO_PIN_8
#define BTN_RESET_PORT  GPIOB

// --- CẤU HÌNH SỐ LƯỢNG HIỆU ỨNG ---
#define MAX_EFFECT_SPI  8
#define MAX_EFFECT_UART 8
#define DEBOUNCE_DELAY  50

// --- LIÊN KẾT BIẾN TỪ MAIN.C ---
// Dùng 'extern' để điều khiển biến đã khai báo bên main.c
extern volatile uint8_t effect_mode_spi;
extern volatile uint8_t effect_mode_uart;

// Struct quản lý nút bấm
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    uint8_t last_state;
    uint8_t is_pressed;
    uint32_t last_time;
} Button_t;

// Khởi tạo trạng thái ban đầu cho 3 nút
static Button_t btn_spi   = {BTN_SPI_PORT,   BTN_SPI_PIN,   0, 0, 0};
static Button_t btn_uart  = {BTN_UART_PORT,  BTN_UART_PIN,  0, 0, 0};
static Button_t btn_reset = {BTN_RESET_PORT, BTN_RESET_PIN, 0, 0, 0};

// Hàm xử lý chung cho 1 nút (để code gọn hơn)
// Trả về 1 nếu nút vừa được nhấn (Rising Edge)
uint8_t check_button(Button_t *btn) {
    uint8_t current_state = HAL_GPIO_ReadPin(btn->port, btn->pin);
    uint8_t pressed = 0;

    // Nếu trạng thái thay đổi, reset bộ đếm thời gian
    if (current_state != btn->last_state) {
        btn->last_time = HAL_GetTick();
    }

    // Nếu trạng thái ổn định trong khoảng debounce
    if ((HAL_GetTick() - btn->last_time) > DEBOUNCE_DELAY) {
        // Nếu nút đang nhấn (Logic 1) và trước đó chưa tính là nhấn
        if (current_state == 1 && btn->is_pressed == 0) {
            btn->is_pressed = 1;
            pressed = 1; // Xác nhận có nhấn
        }
        // Nếu nút nhả ra
        else if (current_state == 0) {
            btn->is_pressed = 0;
        }
    }

    btn->last_state = current_state;
    return pressed;
}

// --- HÀM QUÉT NÚT CHÍNH ---
void button_scan() {

    // 1. XỬ LÝ NÚT SPI (PE0)
    if (check_button(&btn_spi)) {
        effect_mode_spi++;
        // Nếu vượt quá số hiệu ứng, quay về 0 (Breathing)
        if (effect_mode_spi > MAX_EFFECT_SPI) {
            effect_mode_spi = 0;
        }
    }

    // 2. XỬ LÝ NÚT USART (PE1)
    if (check_button(&btn_uart)) {
        effect_mode_uart++;
        if (effect_mode_uart > MAX_EFFECT_UART) {
            effect_mode_uart = 0;
        }
    }

    // 3. XỬ LÝ NÚT RESET (PB8)
    if (check_button(&btn_reset)) {
        perform_system_reset();
    }
}

#endif /* INC_BUTTON_H_ */
