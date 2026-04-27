#include "bsp/m5stack_tab5.h"
#include "esp_log.h"
#include "dashboard_demo.h"
#include "dashboard_ui.h"
#include "ui/dashboard_font.h"
#include "can.h"

#include <stdint.h>
#include <string.h>

static const char *TAG = "main";

#define CAN_ID_ATTITUDE 0x100
#define CAN_ID_HEIGHT   0x101

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
    if (dashboard_font_get(size_px, (dashboard_font_weight_t)weight, &font) == ESP_OK) {
        return font;
    }
    return NULL;
}

typedef struct {
    dashboard_ui_t *ui;
    uint32_t start_ms;
} dashboard_runtime_t;

static dashboard_runtime_t s_runtime;
static volatile int16_t s_rx_pitch = 0;
static volatile int16_t s_rx_roll = 0;
static volatile int16_t s_rx_height_cm = 0;

static bool can_rx_cb(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp)
{
    (void)timestamp;
    if (buffer == NULL) {
        return false;
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
    }
    return false;
}

static void dashboard_timer_cb(lv_timer_t *timer)
{
    dashboard_runtime_t *runtime = lv_timer_get_user_data(timer);
    if (runtime == NULL || runtime->ui == NULL) {
        lv_timer_pause(timer);
        return;
    }

    dashboard_data_t data;
    dashboard_demo_fill(&data, lv_tick_elaps(runtime->start_ms));
    data.pitch_deg = s_rx_pitch;
    data.roll_deg = s_rx_roll;
    data.height_cm = s_rx_height_cm;
    dashboard_ui_set_data(runtime->ui, &data);
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

    dashboard_ui_set_font_callback(tab5_font_get_cb, NULL);

    lv_obj_t *screen = lv_screen_active();
    s_runtime.ui = dashboard_ui_create(screen);
    if (s_runtime.ui == NULL) {
        ESP_LOGE(TAG, "Failed to create dashboard UI");
        bsp_display_unlock();
        return;
    }

    s_runtime.start_ms = lv_tick_get();

    dashboard_data_t data;
    dashboard_demo_fill(&data, 0);
    dashboard_ui_set_data(s_runtime.ui, &data);

    (void)lv_timer_create(dashboard_timer_cb, 50, &s_runtime);
    bsp_display_unlock();

    can_init(can_rx_cb);
    ESP_LOGI(TAG, "CAN initialised, waiting for attitude data...");
}
