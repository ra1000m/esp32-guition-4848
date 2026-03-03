#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_ops.h"
#include "esp_rom_sys.h"
#include "esp_heap_caps.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

static const char *TAG = "ST7701_INIT";

#define LCD_H_RES   480
#define LCD_V_RES   480

/* Pin mapping (Guition 4848S040) */
#define PIN_BL      38
#define PIN_CS      39
#define PIN_SCK     48
#define PIN_SDA     47
#define PIN_PCLK    21
#define PIN_HSYNC   16
#define PIN_VSYNC   17
#define PIN_DE      18

#define SWSPI_DELAY_US 1

/* --- Software SPI 9-bit --- */
static void swspi_init_pins(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL<<PIN_CS) | (1ULL<<PIN_SCK) | (1ULL<<PIN_SDA),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cfg);
    gpio_set_level(PIN_CS, 1);
    gpio_set_level(PIN_SCK, 0);
    gpio_set_level(PIN_SDA, 0);
}

static void swspi_send9(uint16_t word9) {
    gpio_set_level(PIN_CS, 0);
    for (int bit = 8; bit >= 0; --bit) {
        gpio_set_level(PIN_SDA, (word9 >> bit) & 1);
        gpio_set_level(PIN_SCK, 1);
        esp_rom_delay_us(SWSPI_DELAY_US);
        gpio_set_level(PIN_SCK, 0);
        esp_rom_delay_us(SWSPI_DELAY_US);
    }
    gpio_set_level(PIN_CS, 1);
}

static void st7701_cmd(uint8_t cmd) { swspi_send9((0 << 8) | (cmd & 0xFF)); }
static void st7701_data(uint8_t d)  { swspi_send9((1 << 8) | (d & 0xFF)); }

/* --- ТВОИ ДАННЫЕ ИНИЦИАЛИЗАЦИИ (БЕЗ ИЗМЕНЕНИЙ) --- */
static const uint8_t seq_ff_10[] = {0x77,0x01,0x00,0x00,0x10};
static const uint8_t seq_c0_3[] = {0x3B,0x00};
static const uint8_t seq_c1_3[] = {0x0D,0x02};
static const uint8_t seq_c2_3[] = {0x31,0x05};
static const uint8_t seq_cd_1[] = {0x00};
static const uint8_t seq_b0_p[] = {0x00,0x11,0x18,0x0E,0x11,0x06,0x07,0x08,0x07,0x22,0x04,0x12,0x0F,0xAA,0x31,0x18};
static const uint8_t seq_b1_p[] = {0x00,0x11,0x19,0x0E,0x12,0x07,0x08,0x08,0x08,0x22,0x04,0x11,0x11,0xA9,0x32,0x18};
static const uint8_t seq_ff_11[] = {0x77,0x01,0x00,0x00,0x11};
static const uint8_t seq_b0_1[] = {0x60};
static const uint8_t seq_b1_1[] = {0x32};
static const uint8_t seq_b2_1[] = {0x07};
static const uint8_t seq_b3_1[] = {0x80};
static const uint8_t seq_b5_1[] = {0x49};
static const uint8_t seq_b7_1[] = {0x85};
static const uint8_t seq_b8_1[] = {0x21};
static const uint8_t seq_c1_1[] = {0x78};
static const uint8_t seq_c2_1[] = {0x78};
static const uint8_t seq_e0_3[] = {0x00,0x1B,0x02};
static const uint8_t seq_e1_11[] = {0x08,0xA0,0x00,0x00,0x07,0xA0,0x00,0x00,0x00,0x44,0x44};
static const uint8_t seq_e2_12[] = {0x11,0x11,0x44,0x44,0xED,0xA0,0x00,0x00,0xEC,0xA0,0x00,0x00};
static const uint8_t seq_e3_4[] = {0x00,0x00,0x11,0x11};
static const uint8_t seq_e4_2[] = {0x44,0x44};
static const uint8_t seq_e5_16[] = {0x0A,0xE9,0xD8,0xA0,0x0C,0xEB,0xD8,0xA0,0x0E,0xED,0xD8,0xA0,0x10,0xEF,0xD8,0xA0};
static const uint8_t seq_e6_4[] = {0x00,0x00,0x11,0x11};
static const uint8_t seq_e7_2[] = {0x44,0x44};
static const uint8_t seq_e8_16[] = {0x09,0xE8,0xD8,0xA0,0x0B,0xEA,0xD8,0xA0,0x0D,0xEC,0xD8,0xA0,0x0F,0xEE,0xD8,0xA0};
static const uint8_t seq_eb_7[] = {0x02,0x00,0xE4,0xE4,0x88,0x00,0x40};
static const uint8_t seq_ec_2[] = {0x3C,0x00};
static const uint8_t seq_ed_16[] = {0xAB,0x89,0x76,0x54,0x02,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x20,0x45,0x67,0x98,0xBA};
static const uint8_t seq_ff_13[] = {0x77,0x01,0x00,0x00,0x13};
static const uint8_t seq_e5_1[] = {0xE4};
static const uint8_t seq_ff_00[] = {0x77,0x01,0x00,0x00,0x00};
static const uint8_t seq_3a_1[] = {0x50};

typedef struct {
    uint8_t cmd;
    const uint8_t *data;
    uint8_t data_len;
    uint16_t delay_ms_after;
} st7701_seq_entry_t;

static const st7701_seq_entry_t init_seq[] = {
    {0xFF, seq_ff_10, sizeof(seq_ff_10), 0},
    {0xC0, seq_c0_3, sizeof(seq_c0_3), 0},
    {0xC1, seq_c1_3, sizeof(seq_c1_3), 0},
    {0xC2, seq_c2_3, sizeof(seq_c2_3), 0},
    {0xCD, seq_cd_1, sizeof(seq_cd_1), 0},
    {0xB0, seq_b0_p, sizeof(seq_b0_p), 0},
    {0xB1, seq_b1_p, sizeof(seq_b1_p), 0},
    {0xFF, seq_ff_11, sizeof(seq_ff_11), 0},
    {0xB0, seq_b0_1, sizeof(seq_b0_1), 0},
    {0xB1, seq_b1_1, sizeof(seq_b1_1), 0},
    {0xB2, seq_b2_1, sizeof(seq_b2_1), 0},
    {0xB3, seq_b3_1, sizeof(seq_b3_1), 0},
    {0xB5, seq_b5_1, sizeof(seq_b5_1), 0},
    {0xB7, seq_b7_1, sizeof(seq_b7_1), 0},
    {0xB8, seq_b8_1, sizeof(seq_b8_1), 0},
    {0xC1, seq_c1_1, sizeof(seq_c1_1), 0},
    {0xC2, seq_c2_1, sizeof(seq_c2_1), 0},
    {0xE0, seq_e0_3, sizeof(seq_e0_3), 0},
    {0xE1, seq_e1_11, sizeof(seq_e1_11), 0},
    {0xE2, seq_e2_12, sizeof(seq_e2_12), 0},
    {0xE3, seq_e3_4, sizeof(seq_e3_4), 0},
    {0xE4, seq_e4_2, sizeof(seq_e4_2), 0},
    {0xE5, seq_e5_16, sizeof(seq_e5_16), 0},
    {0xE6, seq_e6_4, sizeof(seq_e6_4), 0},
    {0xE7, seq_e7_2, sizeof(seq_e7_2), 0},
    {0xE8, seq_e8_16, sizeof(seq_e8_16), 0},
    {0xEB, seq_eb_7, sizeof(seq_eb_7), 0},
    {0xEC, seq_ec_2, sizeof(seq_ec_2), 0},
    {0xED, seq_ed_16, sizeof(seq_ed_16), 0},
    {0xFF, seq_ff_13, sizeof(seq_ff_13), 0},
    {0xE5, seq_e5_1, sizeof(seq_e5_1), 0},
    {0xFF, seq_ff_00, sizeof(seq_ff_00), 0},
    {0x3A, seq_3a_1, sizeof(seq_3a_1), 0},
    {0x00, NULL, 0, 0}
};

static void run_init_sequence(void) {
    for (int i = 0; init_seq[i].cmd != 0x00; i++) {
        st7701_cmd(init_seq[i].cmd);
        for (int j = 0; j < init_seq[i].data_len; j++) {
            st7701_data(init_seq[i].data[j]);
        }
        if (init_seq[i].delay_ms_after > 0) {
            vTaskDelay(pdMS_TO_TICKS(init_seq[i].delay_ms_after));
        }
    }
    st7701_cmd(0x11); vTaskDelay(pdMS_TO_TICKS(120));
    st7701_cmd(0x29); vTaskDelay(pdMS_TO_TICKS(20));
}

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

static void fill_screen(uint16_t *fb, uint16_t color)
{
    for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) {
        fb[i] = color;
    }
}

static void draw_color_bars(uint16_t *fb)
{
    const uint16_t bars[] = {
        rgb565(255, 255, 255), // White
        rgb565(255, 255, 0),   // Yellow
        rgb565(0, 255, 255),   // Cyan
        rgb565(0, 255, 0),     // Green
        rgb565(255, 0, 255),   // Magenta
        rgb565(255, 0, 0),     // Red
        rgb565(0, 0, 255),     // Blue
        rgb565(0, 0, 0),       // Black
    };
    const int bar_count = sizeof(bars) / sizeof(bars[0]);

    for (int y = 0; y < LCD_V_RES; y++) {
        for (int x = 0; x < LCD_H_RES; x++) {
            int idx = (x * bar_count) / LCD_H_RES;
            fb[y * LCD_H_RES + x] = bars[idx];
        }
    }
}

static void draw_grid(uint16_t *fb)
{
    const int step = 40;
    const uint16_t bg = rgb565(16, 16, 16);
    const uint16_t line = rgb565(220, 220, 220);
    const uint16_t center = rgb565(255, 0, 0);

    for (int y = 0; y < LCD_V_RES; y++) {
        for (int x = 0; x < LCD_H_RES; x++) {
            bool is_grid = (x % step == 0) || (y % step == 0);
            bool is_center = (x == LCD_H_RES / 2) || (y == LCD_V_RES / 2);
            fb[y * LCD_H_RES + x] = is_center ? center : (is_grid ? line : bg);
        }
    }
}

static void draw_gradients(uint16_t *fb)
{
    for (int y = 0; y < LCD_V_RES; y++) {
        for (int x = 0; x < LCD_H_RES; x++) {
            uint8_t r = (uint8_t)(x * 255 / (LCD_H_RES - 1));
            uint8_t g = (uint8_t)(y * 255 / (LCD_V_RES - 1));
            uint8_t b = (uint8_t)((x + y) * 255 / ((LCD_H_RES - 1) + (LCD_V_RES - 1)));
            fb[y * LCD_H_RES + x] = rgb565(r, g, b);
        }
    }
}

/* --- Main App --- */
void app_main(void)
{
    ESP_LOGI(TAG, "Starting main: BL on, ST7701 Init...");

    // Подсветка
    gpio_config_t cfg_bl = { .pin_bit_mask = (1ULL<<PIN_BL), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&cfg_bl);
    gpio_set_level(PIN_BL, 1);

    swspi_init_pins();
    vTaskDelay(pdMS_TO_TICKS(20));
    run_init_sequence();

    ESP_LOGI(TAG, "Init sequence done. Now init RGB driver...");

    esp_lcd_rgb_panel_config_t rgb_config = {
        .data_width = 16,
        .psram_trans_align = 64,
        .num_fbs = 1,
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .pclk_gpio_num = PIN_PCLK,
        .vsync_gpio_num = PIN_VSYNC,
        .hsync_gpio_num = PIN_HSYNC,
        .de_gpio_num = PIN_DE,
        .data_gpio_nums = {
            4, 5, 6, 7, 15,       // B0-B4
            8, 20, 3, 46, 9, 10,  // G0-G5
            11, 12, 13, 14, 0     // R0-R4
        },
        .timings = {
            .pclk_hz = 12000000,
            .h_res = LCD_H_RES,
            .v_res = LCD_V_RES,
            .hsync_pulse_width = 8,
            .hsync_back_porch = 50,
            .hsync_front_porch = 10,
            .vsync_pulse_width = 8,
            .vsync_back_porch = 20,
            .vsync_front_porch = 10,
            .flags.pclk_active_neg = true,
        },
        .flags.fb_in_psram = true,
    };

    esp_lcd_panel_handle_t panel = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&rgb_config, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

    // ПОЛУЧЕНИЕ БУФЕРА (v5.2)
    uint16_t *fb = NULL;
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_fbs(panel, 1, (void **)&fb));

    ESP_LOGI(TAG, "RGB initialized, running LCD screen test...");

    while (1) {
        ESP_LOGI(TAG, "Screen test: RED");
        fill_screen(fb, rgb565(255, 0, 0));
        vTaskDelay(pdMS_TO_TICKS(1500));

        ESP_LOGI(TAG, "Screen test: GREEN");
        fill_screen(fb, rgb565(0, 255, 0));
        vTaskDelay(pdMS_TO_TICKS(1500));

        ESP_LOGI(TAG, "Screen test: BLUE");
        fill_screen(fb, rgb565(0, 0, 255));
        vTaskDelay(pdMS_TO_TICKS(1500));

        ESP_LOGI(TAG, "Screen test: WHITE");
        fill_screen(fb, rgb565(255, 255, 255));
        vTaskDelay(pdMS_TO_TICKS(1500));

        ESP_LOGI(TAG, "Screen test: COLOR BARS");
        draw_color_bars(fb);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Screen test: GRID");
        draw_grid(fb);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Screen test: GRADIENT");
        draw_gradients(fb);
        vTaskDelay(pdMS_TO_TICKS(2500));
    }
}
