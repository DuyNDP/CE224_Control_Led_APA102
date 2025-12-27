/*
 * Config.h
 *
 *  Created on: Dec 24, 2025
 *      Author: admin
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#define NUM_LEDS 30           // How many LEDs are in your strip
#define MAX_BRIGHTNESS 15		// brightness 5 bit
#define LED_FRAME_SIZE 4      // 4 bytes per LED (B, G, R, Brightness)
#define START_FRAME_SIZE 4    // 4 bytes for the start frame
#define END_FRAME_SIZE 4      // 4 bytes for the end frame
#define BUFFER_SIZE (START_FRAME_SIZE + (NUM_LEDS * LED_FRAME_SIZE) + END_FRAME_SIZE)

// --- CẤU HÌNH XỬ LÝ ÂM THANH (FFT) ---
#define FFT_SAMPLES 512              // Số mẫu FFT (phải là lũy thừa của 2: 512, 1024, 2048...)
#define SAMPLING_RATE 64565          // Clock 84MHz / 1301 (ARR=1300) = ~64565 Hz


// --- CẤU HÌNH HIỂN THỊ (SCALING & LIMIT) ---
#define TARGET_MAX_VAL  10000.0f  // Giới hạn hiển thị cường độ (0 - 10k)
#define TARGET_MAX_HZ   30000.0f // Giới hạn hiển thị tần số (0 - 30k)

// Hệ số chia để thu nhỏ giá trị FFT khổng lồ xuống thang 10k.
#define MAG_SCALE_FACTOR 35.0f

// --- CẤU HÌNH LED ---
#define LED_UPDATE_INTERVAL 10  // Thời gian update LED: 500ms

#endif /* INC_CONFIG_H_ */
