/*
 * effect.h
 * Status: UPDATED - Breathing (Case 0) Non-blocking
 */
#ifndef INC_EFFECT_H_
#define INC_EFFECT_H_

#include <stdlib.h>
#include <math.h>
#include "SPI_LED.h"
#include "UART_LED.h"
#include "FFT.h"
#include "knob.h"
#include "button.h"

// --- ĐỊNH NGHĨA ---
#define STRIP_SPI  0
#define STRIP_UART 1
#define MODE_OFF   99
#define ADC_THRESHOLD  100

// --- BIẾN TOÀN CỤC ---
float smoothed_val = 0.0f;
extern volatile uint8_t effect_mode_spi;
extern volatile uint8_t effect_mode_uart;
extern volatile uint32_t brightness;
extern float audio_peak_val;
extern float audio_peak_hz;

// Buffer trạng thái
static uint16_t rain_hues[NUM_LEDS];
static uint8_t  rain_vals[NUM_LEDS];
static uint8_t  fire_heat[NUM_LEDS];

void update_all_strips(void);
void set_strip_color(uint8_t strip_type, uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);
void hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b);
void effect_clear(uint8_t strip_type);
void effect_breathing(uint8_t strip_type);
void effect_vu_meter_smart(float vol, float hz, uint8_t strip_type);
void effect_freq_color(float vol, float hz, uint8_t strip_type);
void effect_rainbow_pulse(float vol, uint8_t strip_type);
void effect_music_rain(float vol, float hz, uint8_t strip_type);
void effect_fire(float vol, uint8_t strip_type);
void effect_center_pulse(float vol, float hz, uint8_t strip_type);
void process_audio_data(void);
void brightness_update(void);
void button_scan(void);
void led_effects_manager(void);
void led_init();
void perform_system_reset(void);

// ============================================================
// 1. CÁC HÀM CƠ BẢN
// ============================================================

void update_all_strips(void) {
    spi_update();
    usart_update();
}

void set_strip_color(uint8_t strip_type, uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
    if (strip_type == STRIP_SPI) spi_set_led(index, r, g, b, brightness);
    else usart_set_led(index, r, g, b, brightness);
}

void hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b) {
    uint8_t f = (h % 60) * 255 / 60;
    uint8_t p = (255 - s) * (uint16_t)v / 255;
    uint8_t q = (255 - f * s / 255) * (uint16_t)v / 255;
    uint8_t t = (255 - (255 - f) * s / 255) * (uint16_t)v / 255;

    switch ((h / 60) % 6) {
        case 0: *r = v; *g = t; *b = p; break;
        case 1: *r = q; *g = v; *b = p; break;
        case 2: *r = p; *g = v; *b = t; break;
        case 3: *r = p; *g = q; *b = v; break;
        case 4: *r = t; *g = p; *b = v; break;
        case 5: *r = v; *g = p; *b = q; break;
    }
}

void effect_clear(uint8_t strip_type) {
    for (int i = 0; i < NUM_LEDS; i++) set_strip_color(strip_type, i, 0, 0, 0, 0);
}

// ============================================================
// 2. CÁC HIỆU ỨNG
// ============================================================

// --- HIỆU ỨNG THỞ (CASE 0 MỚI) ---
// Logic: Fade In -> Fade Out -> Đổi màu
void effect_breathing(uint8_t strip_type) {
    // 1. Tạo sóng thở (Sine wave)
    // Chia 700: Tốc độ thở (càng lớn càng chậm)
    float val = (sin(HAL_GetTick() / 700.0f) + 1.0f) / 2.0f;

    // 2. Gamma Correction (QUAN TRỌNG)
    // Bình phương giá trị val (val * val) giúp đoạn tối sâu hơn, đoạn sáng dịu hơn.
    // Nếu không có dòng này, đèn trông sẽ như luôn sáng và chỉ tối đi 1 chút.
    float deep_breath = val * val;

    // Đảm bảo không tắt hẳn (giữ lại 5% độ sáng thấp nhất để tạo nền)
    if (deep_breath < 0.05f) deep_breath = 0.05f;

    // 3. Tính toán màu RGB dựa trên độ sáng thở
    // Thay vì chỉnh brightness phần cứng, ta chỉnh độ mạnh của màu (Value trong HSV)
    // deep_breath * 255 sẽ cho ra giá trị từ 10 đến 255 -> Cực kỳ mượt
    uint16_t hue = (HAL_GetTick() / 50) % 360;
    uint8_t r, g, b;

    // Tham số thứ 3 (Value) nhận giá trị thở
    hsv_to_rgb(hue, 255, (uint8_t)(deep_breath * 255), &r, &g, &b);

    // 4. Xuất ra LED
    for (int i = 0; i < NUM_LEDS; i++) {
        // Lưu ý: Tham số cuối cùng giữ nguyên biến 'brightness' toàn cục (mức trần)
        // Việc Fade in/out đã được xử lý bên trong r, g, b rồi.
        set_strip_color(strip_type, i, r, g, b, brightness);
    }
}
// 1. Smart VU Meter
void effect_vu_meter_smart(float vol, float hz, uint8_t strip_type) {
    int height = (int)((vol / TARGET_MAX_VAL) * NUM_LEDS);
    if (height > NUM_LEDS) height = NUM_LEDS;

    uint16_t hue = (hz > 0) ? (uint16_t)((hz / 4000.0) * 300) : 0;
    uint8_t r, g, b;
    hsv_to_rgb(hue, 255, 255, &r, &g, &b);

    for (int i = 0; i < NUM_LEDS; i++) {
        if (i < height) set_strip_color(strip_type, i, r, g, b, brightness);
        else set_strip_color(strip_type, i, 0, 0, 0, 0);
    }
}

// 2. Freq Color
void effect_freq_color(float vol, float hz, uint8_t strip_type) {
    int center = NUM_LEDS / 2;
    int width = (int)((vol / TARGET_MAX_VAL) * center);

    uint16_t hue = (hz > 0) ? (uint16_t)((hz / 4000.0) * 300) : 0;
    uint8_t r, g, b;
    hsv_to_rgb(hue, 255, 255, &r, &g, &b);

    for (int i = 0; i < NUM_LEDS; i++) {
        if (i >= (center - width) && i <= (center + width))
             set_strip_color(strip_type, i, r, g, b, brightness);
        else set_strip_color(strip_type, i, 0, 0, 0, 0);
    }
}

// 3. Rainbow Pulse
void effect_rainbow_pulse(float vol, uint8_t strip_type) {
    static uint16_t hue_counter = 0;
    hue_counter++;
    uint8_t r, g, b;
    hsv_to_rgb(hue_counter % 360, 255, 255, &r, &g, &b);

    uint8_t dyn_bright = (uint8_t)((vol / TARGET_MAX_VAL) * brightness);
    if (dyn_bright > brightness) dyn_bright = brightness;

    for (int i = 0; i < NUM_LEDS; i++) set_strip_color(strip_type, i, r, g, b, dyn_bright);
}

// 4. Music Rain
void effect_music_rain(float vol, float hz, uint8_t strip_type) {
    for (int i = 0; i < NUM_LEDS; i++) {
        if (rain_vals[i] > 2) rain_vals[i] -= 2; else rain_vals[i] = 0;
    }
    if (vol > 1000) {
        int pos = rand() % NUM_LEDS;
        rain_vals[pos] = brightness;
        rain_hues[pos] = (uint16_t)((hz / 4000.0) * 300);
    }
    uint8_t r, g, b;
    for (int i = 0; i < NUM_LEDS; i++) {
        if (rain_vals[i] > 0) {
            hsv_to_rgb(rain_hues[i], 255, 255, &r, &g, &b);
            set_strip_color(strip_type, i, r, g, b, rain_vals[i]);
        } else set_strip_color(strip_type, i, 0, 0, 0, 0);
    }
}

// 5. Fire (Hiệu ứng Lửa thực tế)
void effect_fire(float vol, uint8_t strip_type) {
    // 1. Cooling (Làm nguội - Tạo các khoảng tối ngẫu nhiên)
    for (int i = 0; i < NUM_LEDS; i++) {
        int cooldown = (rand() % 10);
        // Trừ nhiệt độ đi, đảm bảo không âm
        if (fire_heat[i] >= cooldown) fire_heat[i] -= cooldown;
        else fire_heat[i] = 0;
    }

    // 2. Drifting (Bốc hơi nhiệt từ dưới lên trên)
    for (int i = NUM_LEDS - 1; i >= 2; i--) {
        // Công thức trung bình cộng để làm mượt chuyển động
        fire_heat[i] = (fire_heat[i - 1] + fire_heat[i - 2] + fire_heat[i - 2]) / 3;
    }

    // 3. Ignition (Mồi lửa theo âm lượng)
    if (vol > 500) { // Ngưỡng kích hoạt
        int ignition = (int)(vol / 150.0f); // Tăng độ nhạy một chút
        if (ignition > 255) ignition = 255;

        // Mồi lửa vào chân (LED 0 và 1)
        // Dùng phép OR hoặc MAX để tránh làm tối đi nếu đang cháy to
        if (ignition > fire_heat[0]) fire_heat[0] = ignition;
        if (ignition > fire_heat[1]) fire_heat[1] = ignition;
    }

    // 4. Mapping (Chuyển nhiệt độ thành màu sắc)
    for (int i = 0; i < NUM_LEDS; i++) {
        uint8_t r, g, b;
        uint8_t heat = fire_heat[i];

        // Dùng chính 'heat' làm độ sáng (Value) cho hàm hsv_to_rgb
        // Điều này giúp ngọn lửa tắt dần tự nhiên ở phía trên

        if (strip_type == STRIP_SPI) {
            // --- FIRE MODE (Đỏ -> Vàng -> Trắng) ---
            // Scale heat (0-255) sang Hue (0-85 là vùng Đỏ-Cam-Vàng)
            uint16_t pixel_hue = (heat / 3);

            // Tham số thứ 3 là 'heat' -> Nhiệt thấp thì đèn tối
            hsv_to_rgb(pixel_hue, 255, heat, &r, &g, &b);
        }
        else {
            // --- ICE MODE (Xanh đậm -> Xanh nhạt -> Trắng) ---
            // Hue 160 (Xanh lam) -> 240 (Xanh dương)
            uint16_t pixel_hue = 160 + (heat / 3);

            // Với băng, có thể giữ Saturation giảm dần để ra màu trắng khi sáng nhất
            hsv_to_rgb(pixel_hue, 255 - (heat/4), heat, &r, &g, &b);
        }

        // Set màu ra LED
        // brightness: Biến toàn cục chỉnh độ sáng tổng (Dimmer)
        set_strip_color(strip_type, i, r, g, b, brightness);
    }
}

// 6. Center Pulse
void effect_center_pulse(float vol, float hz, uint8_t strip_type) {
    int center = NUM_LEDS / 2;
    int len = (int)((vol / TARGET_MAX_VAL) * center);
    uint16_t hue = (uint16_t)((hz / 4000.0) * 300);
    uint8_t r, g, b;
    hsv_to_rgb(hue, 255, 255, &r, &g, &b);
    for (int i = 0; i < NUM_LEDS; i++) {
        if (abs(i - center) < len) set_strip_color(strip_type, i, r, g, b, brightness);
        else set_strip_color(strip_type, i, 0, 0, 0, 0);
    }
}
// ============================================================
// 3. TRÌNH QUẢN LÝ (MANAGER)
// ============================================================

void led_effects_manager(void) {
	process_audio_data();
	brightness_update();
	button_scan();
	float raw_vol = audio_peak_val;
	float raw_hz = audio_peak_hz;
    smoothed_val = (smoothed_val * 0.6f) + (raw_vol * 0.4f);

    // ================= XỬ LÝ DÂY SPI =================
    switch (effect_mode_spi) {
        case 0: effect_breathing(STRIP_SPI); break;
        case 1: effect_vu_meter_smart(smoothed_val, raw_hz, STRIP_SPI); break;
        case 2: effect_freq_color(smoothed_val, raw_hz, STRIP_SPI); break;
        case 3: effect_rainbow_pulse(smoothed_val, STRIP_SPI); break;
        case 4: effect_music_rain(smoothed_val, raw_hz, STRIP_SPI); break;
        case 5: effect_fire(smoothed_val, STRIP_SPI); break;
        case 6: effect_center_pulse(smoothed_val, raw_hz, STRIP_SPI); break;
        case MODE_OFF: effect_clear(STRIP_SPI); break;
        default: effect_breathing(STRIP_SPI); break;
    }

    // ================= XỬ LÝ DÂY USART =================
    switch (effect_mode_uart) {
    	case 0: effect_breathing(STRIP_UART); break;
        case 1: effect_vu_meter_smart(smoothed_val, raw_hz, STRIP_UART); break;
        case 2: effect_freq_color(smoothed_val, raw_hz, STRIP_UART); break;
        case 3: effect_rainbow_pulse(smoothed_val, STRIP_UART); break;
        case 4: effect_music_rain(smoothed_val, raw_hz, STRIP_UART); break;
        case 5: effect_fire(smoothed_val, STRIP_UART); break;
        case 6: effect_center_pulse(smoothed_val, raw_hz, STRIP_UART); break;
        case MODE_OFF: effect_clear(STRIP_UART); break;
        default: effect_breathing(STRIP_UART); break;
    }
    update_all_strips();
}

void led_init() {
    usart_init_buffer();
    spi_init_buffer();
    update_all_strips();
}

// Hàm Reset hệ thống:
void perform_system_reset(void) {
    effect_mode_spi = 99;
    effect_mode_uart = 99;

    effect_clear(STRIP_SPI);
    effect_clear(STRIP_UART);
    update_all_strips();
    HAL_Delay(3000);

    effect_mode_spi = 0;
    effect_mode_uart = 0;
}


#endif /* INC_EFFECT_H_ */
