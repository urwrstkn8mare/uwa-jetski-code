#include "bsp/m5stack_tab5.h"
#include "esp_log.h"
#include "dashboard_demo.h"
#include "dashboard_ui.h"
#include "ui/dashboard_font.h"
#include "can.h"
#include "can_ids.h"

#include <stdint.h>
#include <string.h>
#include "esp_adc/adc_oneshot.h"

static const char *TAG = "main";

/* Force linker to pull lv_ftsystem.c.obj from lvgl library.
 * lv_ftsystem.c defines FT_Stream_Open using LVGL's filesystem API,
 * but it is never referenced from within liblvgl__lvgl.a, so the
 * linker skips it and falls back to FreeType's standard I/O.
 * This global variable initializer forces an unresolved reference
 * that the linker must satisfy from a library. */
extern int FT_Stream_Open(void *stream, const char *filepathname);
static void * __attribute__((used, section(".data"))) force_lv_ftsystem_link_ptr = (void *)FT_Stream_Open;

static const lv_font_t *tab5_font_get_cb(uint16_t size_px, int weight, void *user_data)
{
    (void)user_data;
    const lv_font_t *font = NULL;
    if (dashboard_font_get(size_px, weight, &font) == ESP_OK) {
        return font;
    }
    return NULL;
}

typedef struct {
    dashboard_ui_t *ui;
    uint32_t start_ms;
} dashboard_runtime_t;

static dashboard_runtime_t s_runtime;
static SemaphoreHandle_t s_can_data_mutex;
static int16_t s_rx_pitch = 0;
static int16_t s_rx_roll = 0;
static int16_t s_rx_height_cm = 0;
static int16_t s_rx_servo_deg = 0;
static bool s_got_first_frame = false;
static adc_oneshot_unit_handle_t s_adc_handle = NULL;

static void can_rx_cb(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp)
{
    (void)timestamp;
    if (buffer == NULL) {
        return;
    }
    if (!s_got_first_frame) {
        s_got_first_frame = true;
    }
    if (xSemaphoreTake(s_can_data_mutex, portMAX_DELAY) != pdTRUE) {
        return;
    }
    if (header_id == CAN_ID_ATTITUDE) {
        int16_t pitch, roll;
        memcpy(&pitch, &buffer[0], sizeof(pitch));
        memcpy(&roll, &buffer[2], sizeof(roll));
        s_rx_pitch = pitch;
        s_rx_roll = roll;
    } else if (header_id == CAN_ID_HEIGHT) {
        uint16_t height_u;
        memcpy(&height_u, &buffer[0], sizeof(height_u));
        s_rx_height_cm = (int16_t)height_u;
    } else if (header_id == CAN_ID_SERVO_POS) {
        int16_t servo_deg;
        memcpy(&servo_deg, &buffer[0], sizeof(servo_deg));
        s_rx_servo_deg = servo_deg;
    }
    xSemaphoreGive(s_can_data_mutex);
}

static void dashboard_timer_cb(lv_timer_t *timer)
{
    static bool logged_first = false;
    dashboard_runtime_t *runtime = lv_timer_get_user_data(timer);
    if (runtime == NULL || runtime->ui == NULL) {
        lv_timer_pause(timer);
        return;
    }

    if (!logged_first && s_got_first_frame) {
        logged_first = true;
        ESP_LOGI(TAG, "Receiving CAN data");
    }

    dashboard_data_t data;
    dashboard_demo_fill(&data, lv_tick_elaps(runtime->start_ms));
    if (xSemaphoreTake(s_can_data_mutex, portMAX_DELAY) == pdTRUE) {
        data.pitch_deg = s_rx_pitch;
        data.roll_deg = s_rx_roll;
        data.height_cm = s_rx_height_cm;
        data.rudder_deg = s_rx_servo_deg;
        xSemaphoreGive(s_can_data_mutex);
    }
    dashboard_ui_set_data(runtime->ui, &data);
}

/* Dedicated high-priority task for potentiometer -> CAN.
 * Runs independently of LVGL so a display crash/hang cannot
 * stop control inputs from reaching the bus. */
static void potentiometer_tx_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (s_adc_handle != NULL && s_got_first_frame) {
            int adc_raw = 0;
            adc_channel_t channel;
            adc_unit_t unit;
            if (adc_oneshot_io_to_channel(CONFIG_POTENTIOMETER_GPIO, &unit, &channel) == ESP_OK) {
                if (adc_oneshot_read(s_adc_handle, channel, &adc_raw) == ESP_OK) {
                    int min_val = CONFIG_POTENTIOMETER_MIN_VALUE;
                    int max_val = CONFIG_POTENTIOMETER_MAX_VALUE;
                    int clamped = adc_raw;
                    if (clamped < min_val) clamped = min_val;
                    if (clamped > max_val) clamped = max_val;
                    uint16_t scaled = 0;
                    if (max_val > min_val) {
                        scaled = (uint16_t)(((int64_t)(clamped - min_val) * 100) / (max_val - min_val));
                    }
                    ESP_LOGI(TAG, "Potentiometer raw=%d scaled=%u", adc_raw, scaled);
                    uint8_t can_data[2];
                    memcpy(&can_data[0], &scaled, sizeof(scaled));
                    can_tx(CAN_ID_POTENTIOMETER, can_data, sizeof(can_data));
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    esp_io_expander_handle_t io_expander = bsp_io_expander_init();

    /* Enable EXT5V_EN (E1.P2 on PI4IOE5V6408-1 @ 0x43) to power the external 5V bus */
    if (io_expander != NULL) {
        esp_err_t ret = esp_io_expander_set_output_mode(io_expander, IO_EXPANDER_PIN_NUM_2, IO_EXPANDER_OUTPUT_MODE_PUSH_PULL);
        if (ret == ESP_OK) {
            ret = esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_2, 1);
        }
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "External 5V (EXT5V) enabled successfully");
        } else {
            ESP_LOGE(TAG, "Failed to enable external 5V (EXT5V): %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGE(TAG, "IO expander not initialized, cannot enable EXT5V");
    }

    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_H_RES * CONFIG_BSP_LCD_DRAW_BUF_HEIGHT,
        .double_buffer = true,
        .flags = {
            .buff_dma = 1,
            .buff_spiram = 0,
            .sw_rotate = 1, /* Use software rotation to avoid swap_xy errors */
        },
    };
    /* Increase LVGL task stack: FreeType glyph rendering needs > 7KB */
    cfg.lvgl_port_cfg.task_stack = 24576;

    lv_display_t *disp = bsp_display_start_with_config(&cfg);
    bsp_display_rotate(disp, LV_DISPLAY_ROTATION_270);
    bsp_display_backlight_on();

    /* Turn off unused features to save power */
    bsp_feature_enable(BSP_FEATURE_SPEAKER, false);
    bsp_feature_enable(BSP_FEATURE_CAMERA, false);
    bsp_feature_enable(BSP_FEATURE_USB, false);
    bsp_feature_enable(BSP_FEATURE_WIFI, false);
    ESP_LOGI(TAG, "Unused peripherals (speaker, camera, USB, WiFi) powered off");

    if (!bsp_display_lock(0)) {
        ESP_LOGE(TAG, "Failed to lock display");
        return;
    }

    lv_obj_t *screen = lv_screen_active();
    s_runtime.ui = dashboard_ui_init(screen, tab5_font_get_cb, NULL);
    if (s_runtime.ui == NULL) {
        ESP_LOGE(TAG, "Failed to create dashboard UI");
        bsp_display_unlock();
        return;
    }

    s_runtime.start_ms = lv_tick_get();

    dashboard_data_t data;
    dashboard_demo_fill(&data, 0);
    dashboard_ui_set_data(s_runtime.ui, &data);

    s_can_data_mutex = xSemaphoreCreateMutex();
    if (s_can_data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create CAN data mutex");
        bsp_display_unlock();
        return;
    }

    (void)lv_timer_create(dashboard_timer_cb, 50, &s_runtime);
    bsp_display_unlock();

    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id = ADC_UNIT_2,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
    };
    if (adc_oneshot_new_unit(&adc_cfg, &s_adc_handle) == ESP_OK) {
        adc_oneshot_chan_cfg_t chan_cfg = {
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        adc_channel_t channel;
        adc_unit_t unit;
        if (adc_oneshot_io_to_channel(CONFIG_POTENTIOMETER_GPIO, &unit, &channel) == ESP_OK) {
            if (adc_oneshot_config_channel(s_adc_handle, channel, &chan_cfg) == ESP_OK) {
                ESP_LOGI(TAG, "ADC initialised on GPIO %d", CONFIG_POTENTIOMETER_GPIO);
            }
        }
    } else {
        ESP_LOGE(TAG, "Failed to initialise ADC");
    }

    ESP_ERROR_CHECK(can_init(can_rx_cb));
    ESP_LOGI(TAG, "CAN initialised, waiting for data...");

    /* Spawn input task at higher priority than LVGL so display
     * issues cannot stall control outputs. */
    xTaskCreate(potentiometer_tx_task, "pot_tx", 4096, NULL, 10, NULL);
}
