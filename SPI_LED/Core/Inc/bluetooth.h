/*
 * bluetooth.h
 * Cập nhật: Thêm tính năng báo đèn trạng thái kết nối
 */

#ifndef INC_BLUETOOTH_H_
#define INC_BLUETOOTH_H_

#include "main.h"
#include "effect.h"

extern UART_HandleTypeDef huart2;

// --- ĐỊNH NGHĨA CHÂN (Phải khớp với CubeMX) ---
// Nếu bạn đặt User Label trong CubeMX là BT_STATE và LED_D2 thì main.h đã tự có
// Nếu chưa, hãy định nghĩa thủ công ở đây:
#ifndef BT_STATE_PIN
#define BT_STATE_PIN  GPIO_PIN_4
#define BT_STATE_PORT GPIOC
#endif

#ifndef LED_D2_PIN
#define LED_D2_PIN    GPIO_PIN_6
#define LED_D2_PORT   GPIOA
#endif

volatile char bt_rx_data;

void bluetooth_init(void);
void bluetooth_check_connection(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void bluetooth_init(void) {
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&bt_rx_data, 1);
}

// --- HÀM KIỂM TRA KẾT NỐI (Gọi liên tục trong while(1)) ---
void bluetooth_check_connection(void) {
    // Đọc trạng thái chân STATE của HC-05
    if (HAL_GPIO_ReadPin(BT_STATE_PORT, BT_STATE_PIN) == GPIO_PIN_SET) {
        // HC-05 báo mức 1 -> ĐÃ KẾT NỐI

        // Bật đèn D2 (Lưu ý: Board STM32 thường là Active LOW, tức ghi 0 là sáng)
        // Nếu board bạn ghi 1 là sáng thì đổi thành GPIO_PIN_SET
        HAL_GPIO_WritePin(LED_D2_PORT, LED_D2_PIN, GPIO_PIN_RESET);
    }
    else {
        // HC-05 báo mức 0 -> MẤT KẾT NỐI

        // Tắt đèn D2
        HAL_GPIO_WritePin(LED_D2_PORT, LED_D2_PIN, GPIO_PIN_SET);
    }
}

// --- Callback nhận dữ liệu (Giữ nguyên) ---
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        // Chỉ xử lý các ký tự đơn
        if (bt_rx_data >= '0' && bt_rx_data <= '6') effect_mode_spi = bt_rx_data - '0';
        else if (bt_rx_data == 'x') effect_mode_spi = 99; // Tắt SPI

        else if (bt_rx_data >= 'a' && bt_rx_data <= 'i') effect_mode_uart = bt_rx_data - 'a';
        else if (bt_rx_data == 'y') effect_mode_uart = 99; // Tắt UART

        else if (bt_rx_data == 'r') NVIC_SystemReset(); // Reset mềm

        HAL_UART_Receive_IT(&huart2, (uint8_t*)&bt_rx_data, 1);
    }
}


#endif /* INC_BLUETOOTH_H_ */
