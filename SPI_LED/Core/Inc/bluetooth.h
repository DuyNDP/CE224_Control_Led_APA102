/*
 * bluetooth.h
 * Status: SYNCHRONIZED with Button & Web
 */

#ifndef INC_BLUETOOTH_H_
#define INC_BLUETOOTH_H_

#include "main.h"
#include "effect.h"
#include "button.h" // Import để lấy số lượng MAX_EFFECT

extern UART_HandleTypeDef huart2; // Đảm bảo đúng UART nối với HC05

// --- CẤU HÌNH CHÂN ---
#ifndef BT_STATE_PIN
#define BT_STATE_PIN  GPIO_PIN_4
#define BT_STATE_PORT GPIOC
#endif

#ifndef LED_D2_PIN
#define LED_D2_PIN    GPIO_PIN_6
#define LED_D2_PORT   GPIOA
#endif

// Biến nhận dữ liệu
volatile uint8_t bt_rx_data;

// Cờ báo hiệu Reset (để xử lý trong main tránh treo ngắt)
volatile uint8_t bt_reset_flag = 0;

void bluetooth_init(void) {
    // Nhận 1 byte ngắt
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&bt_rx_data, 1);
}

// --- KIỂM TRA KẾT NỐI ---
void bluetooth_check_connection(void) {
    if (HAL_GPIO_ReadPin(BT_STATE_PORT, BT_STATE_PIN) == GPIO_PIN_SET) {
        HAL_GPIO_WritePin(LED_D2_PORT, LED_D2_PIN, GPIO_PIN_RESET); // Đèn sáng (Active Low)
    } else {
        HAL_GPIO_WritePin(LED_D2_PORT, LED_D2_PIN, GPIO_PIN_SET);   // Đèn tắt
    }
}

// --- CALLBACK XỬ LÝ LỆNH TỪ WEB ---
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {

        // ================= XỬ LÝ CHO DÂY SPI (Dùng SỐ 0-8) =================
        if (bt_rx_data >= '0' && bt_rx_data <= '8') {
            effect_mode_spi = bt_rx_data - '0';
        }
        else if (bt_rx_data == 'x') { // Tắt SPI
            effect_mode_spi = 99;
        }

        // ================= XỬ LÝ CHO DÂY UART (Dùng CHỮ a-i) =================
        // Map: 'a'=0, 'b'=1, 'c'=2 ...
        else if (bt_rx_data >= 'a' && bt_rx_data <= 'i') {
            uint8_t mode = bt_rx_data - 'a'; // 'a' (97) - 'a' (97) = 0
            effect_mode_uart = mode;

            // Kiểm tra giới hạn mode của UART
            if (effect_mode_uart > MAX_EFFECT_UART) {
                // Nếu dây UART không hỗ trợ mode này (ví dụ Mirror 7, Ice 8), có thể map về mode khác
                // Hoặc giữ nguyên nếu bạn đã code support.
                // Ví dụ: effect_mode_uart = 0;
            }
        }
        else if (bt_rx_data == 'y') { // Tắt UART
            effect_mode_uart = 99;
        }

        // ================= HỆ THỐNG =================
        else if (bt_rx_data == 'r') {
             // Reset toàn bộ
             bt_reset_flag = 1;
        }

        // Tiếp tục nhận dữ liệu ngắt
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&bt_rx_data, 1);
    }
}

#endif /* INC_BLUETOOTH_H_ */
