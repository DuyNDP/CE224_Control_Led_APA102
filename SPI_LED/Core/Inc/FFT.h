/*
 * FFT.h
 *
 *  Created on: Dec 24, 2025
 *      Author: admin
 */

#ifndef INC_FFT_H_
#define INC_FFT_H_

#include "main.h"
#include "Config.h"
#include "arm_math.h"
#include "effect.h"

extern ADC_HandleTypeDef hadc1;

// --- Biến cho xử lý âm thanh ---
uint16_t adc_buffer[FFT_SAMPLES];       // Buffer chứa dữ liệu thô từ Mic (DMA nạp vào đây)
float32_t fft_in_buf[FFT_SAMPLES];      // Buffer đầu vào cho hàm FFT (Float)
float32_t fft_out_buf[FFT_SAMPLES];     // Buffer đầu ra (số phức)
float32_t fft_mag_buf[FFT_SAMPLES / 2]; // Buffer biên độ tần số (Kết quả cuối cùng)

volatile uint8_t fft_process_flag = 0;  // Cờ báo hiệu đã thu đủ mẫu
arm_rfft_fast_instance_f32 fft_handler; // Bộ quản lý thư viện FFT

// các biến dùng để debug
int32_t debug_peak_val = 0;
int32_t debug_peak_hz = 0;


// --- BIẾN KẾT QUẢ OUTPUT ---
float audio_peak_val = 0.0f; // Cường độ âm thanh lớn nhất hiện tại (Volume)
float audio_peak_hz  = 0.0f; // Tần số của âm thanh đó (Pitch/Tone)

void audio_init(void);
void process_audio_data(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

void audio_init(void) {
    arm_rfft_fast_init_f32(&fft_handler, FFT_SAMPLES);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, FFT_SAMPLES);
}

void process_audio_data(void) {
    if (fft_process_flag) {
        // 1. Lọc nhiễu DC (Zero centering)
        // Đưa tín hiệu từ 0-4095 về dao động quanh 0
        float32_t avg = 0;
        for (uint16_t i = 0; i < FFT_SAMPLES; i++) {
            avg += (float32_t)adc_buffer[i];
        }
        avg /= (float32_t)FFT_SAMPLES;

        for (uint16_t i = 0; i < FFT_SAMPLES; i++) {
            fft_in_buf[i] = (float32_t)adc_buffer[i] - avg;
        }

        // 2. Thực hiện FFT (Time Domain -> Frequency Domain)
        arm_rfft_fast_f32(&fft_handler, fft_in_buf, fft_out_buf, 0);

        // 3. Tính độ lớn (Magnitude)
        // Hàm này thay thế cho đoạn code: mag = sqrt(re*re + im*im)
        // Kết quả lưu vào fft_mag_buf. Chỉ có FFT_SAMPLES/2 phần tử hợp lệ.
        arm_cmplx_mag_f32(fft_out_buf, fft_mag_buf, FFT_SAMPLES / 2);

        // 4. PHÂN TÍCH TÌM PEAK (Giống cấu trúc bạn yêu cầu)
        // Tìm xem tần số nào đang chiếm ưu thế nhất (To nhất)
        float max_mag = 0.0f;
        uint16_t max_index = 0;

        // Bắt đầu từ k=1 hoặc k=2 để bỏ qua thành phần DC (k=0) thường rất lớn nhưng vô nghĩa
        for (uint16_t k = 2; k < (FFT_SAMPLES / 2); k++) {
            float current_mag = fft_mag_buf[k];

            // Tìm giá trị lớn nhất (Peak Finding)
            if (current_mag > max_mag) {
                max_mag = current_mag;
                max_index = k;
            }
        }

        // 5. Cập nhật kết quả ra biến toàn cục

        // Xử lý Cường độ (VAL): Chia nhỏ xuống và cắt trần (Clamp)
        float final_val = max_mag / MAG_SCALE_FACTOR;
        if (final_val > TARGET_MAX_VAL) final_val = TARGET_MAX_VAL; // Cắt nếu vượt 10k
        if (final_val < 0.0f) final_val = 0.0f;

        // Xử lý Tần số (HZ): Tính theo công thức chuẩn và cắt trần
        float final_hz = (float)max_index * (SAMPLING_RATE / (float)FFT_SAMPLES);
        if (final_hz > TARGET_MAX_HZ) final_hz = TARGET_MAX_HZ; // Cắt nếu vượt 100k
        audio_peak_val = final_val;
        audio_peak_hz  = final_hz;

        // Ép kiểu sang số nguyên để SWV vẽ cho đẹp
        debug_peak_val = (int32_t)audio_peak_val;
        debug_peak_hz  = (int32_t)audio_peak_hz;

        // Gọi hàm chạy hiệu ứng
        led_effects_manager(audio_peak_val, audio_peak_hz);

        // 6. Reset cờ
        fft_process_flag = 0;
        extern ADC_HandleTypeDef hadc1;
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, FFT_SAMPLES);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        HAL_ADC_Stop_DMA(&hadc1);
        fft_process_flag = 1;
    }
}

#endif /* INC_FFT_H_ */
