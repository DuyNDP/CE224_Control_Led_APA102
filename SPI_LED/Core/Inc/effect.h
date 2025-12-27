/*
 * effect.h
 * Status: NON-BLOCKING & SYNCHRONIZED
 */
#ifndef INC_EFFECT_H_
#define INC_EFFECT_H_

#include <stdlib.h>
#include <math.h>
#include "SPI_LED.h"
#include "UART_LED.h"

// --- BIẾN TOÀN CỤC ---
float smoothed_val = 0.0f;
uint16_t rainbow_hue = 0;
extern volatile uint8_t effect_mode_spi;
extern volatile uint8_t effect_mode_uart;

// Buffer cho hiệu ứng Mưa & Lửa
static uint16_t rain_hues[NUM_LEDS];
static uint8_t  rain_vals[NUM_LEDS];
static uint8_t  fire_heat[NUM_LEDS];

// Buffer màu tạm cho Matrix Rain
uint8_t buffer_r[NUM_LEDS];
uint8_t buffer_g[NUM_LEDS];
uint8_t buffer_b[NUM_LEDS];

// Hàm cập nhật dữ liệu ra cả 2 cổng
void update_all_strips(void) {
    spi_update();
    usart_update();
}

void led_init()
{
	spi_init_buffer();
	usart_init_buffer();
	update_all_strips();
}

// --- HÀM SET MÀU CHUNG (QUAN TRỌNG) ---
// Hàm này sẽ gán màu cho cả buffer SPI và buffer USART cùng lúc
void set_pixel_color(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t bright) {
    spi_set_led(index, r, g, b, bright);
    usart_set_led(index, r, g, b, bright);
}

// --- HÀM CHUYỂN ĐỔI MÀU (HSV -> RGB) ---
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

// --- HIỆU ỨNG KHỞI ĐỘNG (BLOCKING) ---
void effect_startup_breathing() {
    uint16_t hue = 0;
    uint8_t r, g, b;
    // Fade In
    for (int val = 0; val <= 255; val += 5) {
    	hsv_to_rgb(hue, 255, val, &r, &g, &b);
    	// Dùng MAX_BRIGHTNESS/2 để không quá chói lúc khởi động
        for (int i = 0; i < NUM_LEDS; i++) spi_set_led(i, r, g, b, MAX_BRIGHTNESS);
        	spi_update();
            HAL_Delay(30);
    }
    // Fade Out
    for (int val = 255; val >= 0; val -= 5) {
    	hsv_to_rgb(hue, 255, val, &r, &g, &b);
        for (int i = 0; i < NUM_LEDS; i++) set_pixel_color(i, r, g, b, MAX_BRIGHTNESS);
        	update_all_strips(); // Gửi cả SPI và UART
            HAL_Delay(30);
    }
    hue += 60;
    if (hue >= 360) hue = 0;
}

// --- 1. VU METER SMART ---
void effect_vu_meter_smart(float vol, float hz) {
    static uint32_t last_tick = 0;
    if (HAL_GetTick() - last_tick < 10) return;
    last_tick = HAL_GetTick();

    int num_lit = (int)((vol / TARGET_MAX_VAL) * NUM_LEDS);
    if (num_lit > NUM_LEDS) num_lit = NUM_LEDS;

    uint16_t base_hue = 0;
    if (hz < 500.0f) base_hue = 0;        // Bass: Đỏ
    else if (hz < 2000.0f) base_hue = 30; // Mid: Cam
    else if (hz < 5000.0f) base_hue = 120;// High: Xanh lá
    else base_hue = 200;                  // Treble: Xanh dương

    uint8_t r, g, b;
    for (int i = 0; i < NUM_LEDS; i++) {
        if (i < num_lit) {
            uint16_t pixel_hue = (base_hue + (i * 2)) % 360;
            hsv_to_rgb(pixel_hue, 255, 255, &r, &g, &b);
            spi_set_led(i, r, g, b, MAX_BRIGHTNESS);
        } else {
            spi_set_led(i, 0, 0, 0, 0);
        }
    }
    spi_update();
}

// --- 2. FREQUENCY COLOR ---
void effect_freq_color(float vol, float hz) {
    static uint32_t last_tick = 0;
    if (HAL_GetTick() - last_tick < 15) return;
    last_tick = HAL_GetTick();

    uint8_t r = 0, g = 0, b = 0;
    // Ngưỡng lọc nhiễu 100.0f
    if (vol > 100.0f) {
        if (hz < 300.0f)       { r = 255; g = 0;   b = 0; }
        else if (hz < 1000.0f) { r = 255; g = 100; b = 0; }
        else if (hz < 3000.0f) { r = 0;   g = 255; b = 0; }
        else                   { r = 0;   g = 0;   b = 255; }

        int width = (int)((vol / TARGET_MAX_VAL) * (NUM_LEDS / 2));
        int center = NUM_LEDS / 2;

        for (int i = 0; i < NUM_LEDS; i++) {
            if (i >= center - width && i <= center + width)
                spi_set_led(i, r, g, b, MAX_BRIGHTNESS);
            else
                spi_set_led(i, 0, 0, 0, 0);
        }
    } else {
        for(int i=0; i<NUM_LEDS; i++) spi_set_led(i, 0,0,0,0);
    }
    spi_update();
}

// --- 3. RAINBOW PULSE ---
void effect_rainbow_pulse(float vol) {
    static uint32_t last_tick = 0;
    if (HAL_GetTick() - last_tick < 20) return;
    last_tick = HAL_GetTick();

    rainbow_hue += 2;
    if (rainbow_hue >= 360) rainbow_hue = 0;

    // Map volume to dynamic brightness (ít nhất là 1, tối đa là MAX_BRIGHTNESS)
    uint8_t dyn_bright = (uint8_t)((vol / TARGET_MAX_VAL) * MAX_BRIGHTNESS);
    if (dyn_bright > MAX_BRIGHTNESS) dyn_bright = MAX_BRIGHTNESS;
    if (dyn_bright < 1) dyn_bright = 1;

    uint8_t r, g, b;
    for (int i = 0; i < NUM_LEDS; i++) {
        uint16_t pixel_hue = (rainbow_hue + (i * 360 / NUM_LEDS)) % 360;
        hsv_to_rgb(pixel_hue, 255, 255, &r, &g, &b);
        spi_set_led(i, r, g, b, dyn_bright);
    }
    spi_update();
}

// --- 4. MUSIC RAIN SPARKLE ---
void effect_music_rain(float vol, float hz) {
    static uint32_t last_tick = 0;
    if (HAL_GetTick() - last_tick < 15) return;
    last_tick = HAL_GetTick();

    int fade_speed = 10;
    for (int i = 0; i < NUM_LEDS; i++) {
        if (rain_vals[i] > fade_speed) rain_vals[i] -= fade_speed;
        else rain_vals[i] = 0;
    }

    int drops_to_spawn = 0;
    if (vol > 100.0f) drops_to_spawn = (int)((vol / TARGET_MAX_VAL) * 5); // Giảm hệ số spawn xuống chút cho đỡ rối
    if (drops_to_spawn > NUM_LEDS / 4) drops_to_spawn = NUM_LEDS / 4;

    uint16_t current_hue = 0;
    if (hz > 0) current_hue = (uint16_t)((hz / 4000.0) * 300);

    for (int k = 0; k < drops_to_spawn; k++) {
        int pos = rand() % NUM_LEDS;
        if (rain_vals[pos] < 50) {
            rain_hues[pos] = current_hue;
            rain_vals[pos] = 255;
        }
    }

    for (int i = 0; i < NUM_LEDS; i++) {
        if (rain_vals[i] > 0) {
            uint8_t r, g, b;
            hsv_to_rgb(rain_hues[i], 255, rain_vals[i], &r, &g, &b);
            spi_set_led(i, r, g, b, MAX_BRIGHTNESS);
        } else {
            spi_set_led(i, 0, 0, 0, 0);
        }
    }
    spi_update();
}

// --- 5. FALLING RAIN MATRIX ---
void effect_falling_rain(float vol, float hz, int speed_ms) {
    static uint32_t last_tick = 0;
    if (HAL_GetTick() - last_tick < speed_ms) return;
    last_tick = HAL_GetTick();

    for (int i = NUM_LEDS - 1; i > 0; i--) {
        buffer_r[i] = (uint8_t)(buffer_r[i-1] * 0.8); // Làm mờ nhanh hơn chút (0.8)
        buffer_g[i] = (uint8_t)(buffer_g[i-1] * 0.8);
        buffer_b[i] = (uint8_t)(buffer_b[i-1] * 0.8);
    }

    if (vol > 150.0f) {
        uint8_t r, g, b;
        uint16_t hue = 0;
        if(hz > 0) hue = (uint16_t)((hz / 4000.0) * 300);
        hsv_to_rgb(hue, 255, 255, &r, &g, &b);
        buffer_r[0] = r; buffer_g[0] = g; buffer_b[0] = b;
    } else {
        buffer_r[0] = 0; buffer_g[0] = 0; buffer_b[0] = 0;
    }

    for (int i = 0; i < NUM_LEDS; i++) {
        spi_set_led(i, buffer_r[i], buffer_g[i], buffer_b[i], MAX_BRIGHTNESS);
    }
    spi_update();
}

// --- 6. FIRE EFFECT ---
void effect_fire(float vol) {
    static uint32_t last_tick = 0;
    if (HAL_GetTick() - last_tick < 20) return;
    last_tick = HAL_GetTick();

    for( int i = 0; i < NUM_LEDS; i++) {
        int cooldown = (rand() % 10) + 2;
        if(cooldown > fire_heat[i]) fire_heat[i] = 0;
        else fire_heat[i] = fire_heat[i] - cooldown;
    }

    for( int k = NUM_LEDS - 1; k >= 3; k--) {
        fire_heat[k] = (fire_heat[k - 1] + fire_heat[k - 2] + fire_heat[k - 2]) / 3;
    }

    if( vol > 50.0f ) {
        int heat_input = (int)((vol / TARGET_MAX_VAL) * 255);
        if(heat_input > 255) heat_input = 255;
        fire_heat[0] = heat_input;
        fire_heat[1] = (uint8_t)(heat_input * 0.8);
    }

    for( int j = 0; j < NUM_LEDS; j++) {
        uint8_t temperature = fire_heat[j];
        uint8_t r, g, b;
        uint8_t t192 = (uint8_t)((temperature / 255.0) * 191);
        uint8_t heatramp = t192 & 0x3F; heatramp <<= 2;

        if( t192 > 0x80) { r = 255; g = 255; b = heatramp; }
        else if( t192 > 0x40 ) { r = 255; g = heatramp; b = 0; }
        else { r = heatramp; g = 0; b = 0; }

        spi_set_led(j, r, g, b, MAX_BRIGHTNESS);
    }
    spi_update();
}

// --- 7. CENTER PULSE ---
void effect_center_pulse(float vol, float hz) {
    static uint32_t last_tick = 0;
    if (HAL_GetTick() - last_tick < 10) return;
    last_tick = HAL_GetTick();

    int center = NUM_LEDS / 2;
    int width = (int)((vol / TARGET_MAX_VAL) * center);
    if (width > center) width = center;

    uint16_t hue = 0;
    if (hz > 0) hue = (uint16_t)((hz / 4000.0) * 300);
    uint8_t r, g, b;
    hsv_to_rgb(hue, 255, 255, &r, &g, &b);

    for (int i = 0; i < NUM_LEDS; i++) {
        if (i >= (center - width) && i <= (center + width))
            spi_set_led(i, r, g, b, MAX_BRIGHTNESS);
        else
            spi_set_led(i, 0, 0, 0, 0);
    }
    spi_update();
}

// --- QUẢN LÝ CHÍNH ---
void led_effects_manager(float raw_vol, float raw_hz) {
    // 1. Lọc tín hiệu (Smoothing)
    smoothed_val = (smoothed_val * 0.6f) + (raw_vol * 0.4f);

    // 2. Chọn hiệu ứng
    switch (effect_mode_spi) {
    	case 0: effect_vu_meter_smart(smoothed_val, raw_hz); break;
        case 1: effect_vu_meter_smart(smoothed_val, raw_hz); break;
        case 2: effect_freq_color(smoothed_val, raw_hz); break;
        case 3: effect_rainbow_pulse(smoothed_val); break;
        case 4: effect_music_rain(smoothed_val, raw_hz); break;
        case 5: effect_falling_rain(smoothed_val, raw_hz, 40); break;
        case 6: effect_fire(smoothed_val); break;
        case 7: effect_center_pulse(smoothed_val, raw_hz); break;
        default:
        	effect_vu_meter_smart(smoothed_val, raw_hz);
        	break;
    }
    update_all_strips();
}

// --- HÀM RESET TOÀN BỘ HỆ THỐNG ---
void reset_system_effects(void) {
    // 1. Đưa mode về mặc định (hoặc mode bạn thích nhất)
    effect_mode_spi = 0;
    effect_mode_uart = 0;

    // 2. Xóa sạch dữ liệu trong Buffer (về màu đen 0,0,0)
    for (int i = 0; i < NUM_LEDS; i++) {
        // Xóa buffer SPI
        spi_set_led(i, 0, 0, 0, 0);
        // Xóa buffer USART
        usart_set_led(i, 0, 0, 0, 0);
    }

    // 3. Đẩy dữ liệu đen ra đèn ngay lập tức
    update_all_strips();
}

#endif /* INC_EFFECT_H_ */
