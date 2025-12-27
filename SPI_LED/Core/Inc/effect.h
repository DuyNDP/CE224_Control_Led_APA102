/*
 * effect.h
 * Status: FIXED - Added hsv_to_rgb & Reordered functions
 */
#ifndef INC_EFFECT_H_
#define INC_EFFECT_H_

#include <stdlib.h>
#include <math.h>
#include "SPI_LED.h"
#include "UART_LED.h"

// --- ĐỊNH NGHĨA ---
#define STRIP_SPI  0
#define STRIP_UART 1
#define MODE_OFF   99

// --- BIẾN TOÀN CỤC ---
float smoothed_val = 0.0f;
extern volatile uint8_t effect_mode_spi;
extern volatile uint8_t effect_mode_uart;

// Buffer trạng thái
static uint16_t rain_hues[NUM_LEDS];
static uint8_t  rain_vals[NUM_LEDS];
static uint8_t  fire_heat[NUM_LEDS];

// ============================================================
// 1. CÁC HÀM CƠ BẢN (PHẢI ĐẶT Ở ĐẦU)
// ============================================================

// Hàm đẩy dữ liệu ra phần cứng (Sửa lỗi undefined reference cho button.h)
void update_all_strips(void) {
    spi_update();
    usart_update();
}

// Hàm set màu trung gian
void set_pixel_color(uint8_t strip_type, uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
    if (strip_type == STRIP_SPI) spi_set_led(index, r, g, b, brightness);
    else usart_set_led(index, r, g, b, brightness);
}

// Hàm chuyển đổi HSV sang RGB (Sửa lỗi undefined reference cho effect)
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

// Hàm xóa LED
void effect_clear(uint8_t strip_type) {
    for (int i = 0; i < NUM_LEDS; i++) set_pixel_color(strip_type, i, 0, 0, 0, 0);
}

// ============================================================
// 2. CÁC HIỆU ỨNG (LOGIC CHÍNH)
// ============================================================

// 0. Breathing
void effect_breathing(uint8_t strip_type) {
    float val = (sin(HAL_GetTick() / 500.0f) + 1.0f) / 2.0f;
    uint8_t brightness = (uint8_t)(val * MAX_BRIGHTNESS);
    if(brightness < 2) brightness = 2;

    for (int i = 0; i < NUM_LEDS; i++) {
        set_pixel_color(strip_type, i, 0, 255, 255, brightness);
    }
}

// 1. Smart VU Meter
void effect_vu_meter_smart(float vol, float hz, uint8_t strip_type) {
    int height = (int)((vol / TARGET_MAX_VAL) * NUM_LEDS);
    if (height > NUM_LEDS) height = NUM_LEDS;

    uint16_t hue = (hz > 0) ? (uint16_t)((hz / 4000.0) * 300) : 0;
    uint8_t r, g, b;
    hsv_to_rgb(hue, 255, 255, &r, &g, &b); // Đã có hàm hsv_to_rgb ở trên

    for (int i = 0; i < NUM_LEDS; i++) {
        if (i < height) set_pixel_color(strip_type, i, r, g, b, MAX_BRIGHTNESS);
        else set_pixel_color(strip_type, i, 0, 0, 0, 0);
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
             set_pixel_color(strip_type, i, r, g, b, MAX_BRIGHTNESS);
        else set_pixel_color(strip_type, i, 0, 0, 0, 0);
    }
}

// 3. Rainbow Pulse
void effect_rainbow_pulse(float vol, uint8_t strip_type) {
    static uint16_t hue_counter = 0;
    hue_counter++;
    uint8_t r, g, b;
    hsv_to_rgb(hue_counter % 360, 255, 255, &r, &g, &b);
    uint8_t dyn_bright = (uint8_t)((vol / TARGET_MAX_VAL) * MAX_BRIGHTNESS);
    if (dyn_bright > MAX_BRIGHTNESS) dyn_bright = MAX_BRIGHTNESS;

    for (int i = 0; i < NUM_LEDS; i++) set_pixel_color(strip_type, i, r, g, b, dyn_bright);
}

// 4. Music Rain
void effect_music_rain(float vol, float hz, uint8_t strip_type) {
    for (int i = 0; i < NUM_LEDS; i++) {
        if (rain_vals[i] > 2) rain_vals[i] -= 2; else rain_vals[i] = 0;
    }
    if (vol > 1000) {
        int pos = rand() % NUM_LEDS;
        rain_vals[pos] = MAX_BRIGHTNESS;
        rain_hues[pos] = (uint16_t)((hz / 4000.0) * 300);
    }
    uint8_t r, g, b;
    for (int i = 0; i < NUM_LEDS; i++) {
        if (rain_vals[i] > 0) {
            hsv_to_rgb(rain_hues[i], 255, 255, &r, &g, &b);
            set_pixel_color(strip_type, i, r, g, b, rain_vals[i]);
        } else set_pixel_color(strip_type, i, 0, 0, 0, 0);
    }
}

// 5. Fire (SPI = Lửa, USART = Băng)
void effect_fire(float vol, uint8_t strip_type) {
    // 1. Làm nguội
    for (int i = 0; i < NUM_LEDS; i++) {
        int cooldown = (rand() % 10);
        if (fire_heat[i] >= cooldown) fire_heat[i] -= cooldown; else fire_heat[i] = 0;
    }
    // 2. Bốc hơi
    for (int i = NUM_LEDS - 1; i >= 3; i--) {
        fire_heat[i] = (fire_heat[i - 1] + fire_heat[i - 2] + fire_heat[i - 2]) / 3;
    }
    // 3. Mồi lửa
    if (vol > 500) {
        int ignition = (int)(vol / 200);
        if (ignition > 255) ignition = 255;
        fire_heat[0] = ignition; fire_heat[1] = ignition;
    }

    // 4. Mapping
    for (int i = 0; i < NUM_LEDS; i++) {
        uint8_t r, g, b;
        if (strip_type == STRIP_SPI) {
            uint16_t pixel_hue = (uint8_t)((fire_heat[i] / 255.0) * 191) / 3;
            hsv_to_rgb(pixel_hue, 255, 255, &r, &g, &b);
        } else {
            uint16_t pixel_hue = 160 + (uint8_t)((fire_heat[i] / 255.0) * 80);
            hsv_to_rgb(pixel_hue, 255, 255, &r, &g, &b);
        }
        set_pixel_color(strip_type, i, r, g, b, MAX_BRIGHTNESS);
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
        if (abs(i - center) < len) set_pixel_color(strip_type, i, r, g, b, MAX_BRIGHTNESS);
        else set_pixel_color(strip_type, i, 0, 0, 0, 0);
    }
}

// 7. Mirror VU
void effect_mirror_vu(float vol, float hz) {
    int num_lit = (int)((vol / TARGET_MAX_VAL) * NUM_LEDS);
    if (num_lit > NUM_LEDS) num_lit = NUM_LEDS;
    uint16_t hue = (hz > 0) ? (uint16_t)((hz / 4000.0) * 300) : 0;
    uint8_t r, g, b;
    hsv_to_rgb(hue, 255, 255, &r, &g, &b);

    for (int i = 0; i < NUM_LEDS; i++) {
        if (i < num_lit) {
            spi_set_led(i, r, g, b, MAX_BRIGHTNESS);
            usart_set_led(NUM_LEDS - 1 - i, r, g, b, MAX_BRIGHTNESS);
        } else {
            spi_set_led(i, 0, 0, 0, 0);
            usart_set_led(NUM_LEDS - 1 - i, 0, 0, 0, 0);
        }
    }
}

// 8. Fire and Ice (SPI Lửa - UART Băng)
void effect_fire_ice(float vol) {
    int height = (int)((vol / TARGET_MAX_VAL) * NUM_LEDS);
    if (height > NUM_LEDS) height = NUM_LEDS;
    uint8_t r, g, b;

    for (int i = 0; i < NUM_LEDS; i++) {
        if (i < height) {
            // SPI: Lửa
            uint16_t fire_hue = (i * 60 / NUM_LEDS);
            hsv_to_rgb(fire_hue, 255, 255, &r, &g, &b);
            spi_set_led(i, r, g, b, MAX_BRIGHTNESS);

            // UART: Băng
            uint16_t ice_hue = 180 + (i * 60 / NUM_LEDS);
            hsv_to_rgb(ice_hue, 255, 255, &r, &g, &b);
            usart_set_led(i, r, g, b, MAX_BRIGHTNESS);
        } else {
            spi_set_led(i, 0, 0, 0, 0);
            usart_set_led(i, 0, 0, 0, 0);
        }
    }
}

// ============================================================
// 3. TRÌNH QUẢN LÝ (MANAGER)
// ============================================================

void led_effects_manager(float raw_vol, float raw_hz) {
    // 1. Lọc tín hiệu
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

        case 7: effect_mirror_vu(smoothed_val, raw_hz); break;
        case 8: effect_fire_ice(smoothed_val); break;

        case MODE_OFF: effect_clear(STRIP_SPI); break;
        default: effect_breathing(STRIP_SPI); break;
    }

    // ================= XỬ LÝ DÂY USART =================
    if (effect_mode_spi != 7 && effect_mode_spi != 8) {
        switch (effect_mode_uart) {
            case 0: effect_breathing(STRIP_UART); break;
            case 1: effect_vu_meter_smart(smoothed_val, raw_hz, STRIP_UART); break;
            case 2: effect_freq_color(smoothed_val, raw_hz, STRIP_UART); break;
            case 3: effect_rainbow_pulse(smoothed_val, STRIP_UART); break;
            case 4: effect_music_rain(smoothed_val, raw_hz, STRIP_UART); break;
            case 5: effect_fire(smoothed_val, STRIP_UART); break; // Tự động thành Băng nhờ logic hàm fire
            case 6: effect_center_pulse(smoothed_val, raw_hz, STRIP_UART); break;

            case 7: effect_mirror_vu(smoothed_val, raw_hz); break;
            case 8: effect_fire_ice(smoothed_val); break;

            case MODE_OFF: effect_clear(STRIP_UART); break;
            default: effect_breathing(STRIP_UART); break;
        }
    }

    // 3. Đẩy dữ liệu ra phần cứng
    update_all_strips(); // Đã được định nghĩa ở đầu file
}

void led_init() {
    for (int i = 0; i < BUFFER_SIZE; i++) {
        spi_led_buffer[i] = 0x00;
        usart_led_buffer[i] = 0x00;
    }
    update_all_strips();
}

// Thêm vào effect.h
void perform_system_reset(void) {
    // 1. Tắt đèn
    effect_mode_spi = 99;
    effect_mode_uart = 99;

    // 2. Xóa dữ liệu
    effect_clear(0);
    effect_clear(1);
    update_all_strips();

    // 3. Chờ 3 giây
    HAL_Delay(3000);

    // 4. Về mặc định
    effect_mode_spi = 0;
    effect_mode_uart = 0;
}

#endif /* INC_EFFECT_H_ */
