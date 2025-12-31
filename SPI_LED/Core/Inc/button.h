/*
 * button.h
 * Status: FINAL - Tích hợp hàm kiểm tra Reset cứng
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include "main.h"

// --- LIÊN KẾT BIẾN TỪ MAIN ---
extern UART_HandleTypeDef huart2;

// --- CẤU HÌNH PIN ---
#define BTN_SPI_PIN     GPIO_PIN_0
#define BTN_SPI_PORT    GPIOE
#define BTN_UART_PIN    GPIO_PIN_1
#define BTN_UART_PORT   GPIOE

#define DEBOUNCE_DELAY  50      // 50ms chống rung
#define MAX_EFFECTS     7       // 0 đến 8

// --- BIẾN TRẠNG THÁI HIỆU ỨNG ---
volatile uint8_t effect_mode_spi  = 0;
volatile uint8_t effect_mode_uart = 0;

// --- STRUCT QUẢN LÝ NÚT ---
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    uint8_t last_state;
    uint32_t last_time;
    uint8_t is_pressed;
} Button_t;

Button_t btn_spi =  {BTN_SPI_PORT,  BTN_SPI_PIN,  0, 0, 0};
Button_t btn_uart = {BTN_UART_PORT, BTN_UART_PIN, 0, 0, 0};

void button_init(void);
void check_system_reset_cause(void);
void button_scan(void);

void button_init(void) {
	HAL_Delay(1000);

	// 2. Đọc trạng thái thực tế của nút và GÁN THẲNG vào trạng thái cũ
	// Nút SPI
	if (HAL_GPIO_ReadPin(BTN_SPI_PORT, BTN_SPI_PIN) == GPIO_PIN_SET) {
		btn_spi.last_state = 1; // Nếu đang bị giữ hoặc nhiễu mức 1, coi như đã biết
	    btn_spi.is_pressed = 1; // Đánh dấu là đang nhấn để không trigger event
	} else {
		btn_spi.last_state = 0;
		btn_spi.is_pressed = 0;
	}

    // Nút UART
	if (HAL_GPIO_ReadPin(BTN_UART_PORT, BTN_UART_PIN) == GPIO_PIN_SET) {
		btn_uart.last_state = 1;
		btn_uart.is_pressed = 1;
	} else {
		btn_uart.last_state = 0;
		btn_uart.is_pressed = 0;
	}

    // 3. Reset cứng biến Mode về 0 lần cuối cùng
	effect_mode_spi = 0;
	effect_mode_uart = 0;

	check_system_reset_cause();
}

// ============================================================
// KIỂM TRA NGUYÊN NHÂN RESET
// ============================================================
void check_system_reset_cause(void) {
    // Kiểm tra cờ Reset do chân NRST (Nút cứng) hoặc POR (Cắm điện)
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) || __HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
    {
        // 1. Chờ 1 chút để điện áp và Bluetooth ổn định
        HAL_Delay(100);

        // 2. Gửi lệnh 'r' lên Web để Web tắt hết đèn nút bấm
        uint8_t tx_reset = 'r';
        HAL_UART_Transmit(&huart2, &tx_reset, 1, 100);

        // 3. Xóa cờ báo Reset để lần sau không bị nhầm
        __HAL_RCC_CLEAR_RESET_FLAGS();
    }
}

// ============================================================
// HÀM QUÉT NÚT
// ============================================================
void button_scan(void) {
    uint32_t current_time = HAL_GetTick();

    // --- 1. NÚT SPI ---
    uint8_t read_spi = HAL_GPIO_ReadPin(btn_spi.port, btn_spi.pin);
    if (read_spi != btn_spi.last_state) btn_spi.last_time = current_time;

    if ((current_time - btn_spi.last_time) > DEBOUNCE_DELAY) {
        if (read_spi == 1 && btn_spi.is_pressed == 0) {
            btn_spi.is_pressed = 1;

            effect_mode_spi++;
            if (effect_mode_spi >= MAX_EFFECTS) effect_mode_spi = 0;

            // Gửi '0' -> '8'
            uint8_t tx_char = effect_mode_spi + '0';
            HAL_UART_Transmit(&huart2, &tx_char, 1, 10);
        }
        if (read_spi == 0) btn_spi.is_pressed = 0;
    }
    btn_spi.last_state = read_spi;

    // --- 2. NÚT UART ---
    uint8_t read_uart = HAL_GPIO_ReadPin(btn_uart.port, btn_uart.pin);
    if (read_uart != btn_uart.last_state) btn_uart.last_time = current_time;

    if ((current_time - btn_uart.last_time) > DEBOUNCE_DELAY) {
        if (read_uart == 1 && btn_uart.is_pressed == 0) {
            btn_uart.is_pressed = 1;

            effect_mode_uart++;
            if (effect_mode_uart >= MAX_EFFECTS) effect_mode_uart = 0;

            // Gửi 'a' -> 'i'
            uint8_t tx_char = effect_mode_uart + 'a';
            HAL_UART_Transmit(&huart2, &tx_char, 1, 10);
        }
        if (read_uart == 0) btn_uart.is_pressed = 0;
    }
    btn_uart.last_state = read_uart;
}

#endif /* INC_BUTTON_H_ */
