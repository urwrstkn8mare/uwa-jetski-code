#include "dashboard_font.h"

#include "esp_log.h"
#include "esp_lv_fs.h"
#include "esp_mmap_assets.h"
#include "mmap_generate_fonts.h"

#if __has_include("src/libs/freetype/lv_freetype.h")
#include "src/libs/freetype/lv_freetype.h"
#elif __has_include("src/extra/libs/freetype/lv_freetype.h")
#include "src/extra/libs/freetype/lv_freetype.h"
#else
#error "LVGL FreeType header not found"
#endif

#define DASHBOARD_FONT_DRIVE_LETTER 'F'
#define DASHBOARD_FONT_PARTITION_LABEL "fonts"
#define DASHBOARD_FONT_CACHE_MAX 16

static const char *TAG = "dashboard_font";

typedef struct {
    uint16_t size_px;
    dashboard_font_weight_t weight;
    lv_font_t *font;
} font_cache_entry_t;

static mmap_assets_handle_t s_font_assets;
static esp_lv_fs_handle_t s_font_fs;
static bool s_assets_ready;
static font_cache_entry_t s_cache[DASHBOARD_FONT_CACHE_MAX];
static size_t s_cache_count;

static esp_err_t ensure_assets_mounted(void)
{
    if (s_assets_ready) {
        return ESP_OK;
    }

    const mmap_assets_config_t mmap_cfg = {
        .partition_label = DASHBOARD_FONT_PARTITION_LABEL,
        .max_files = MMAP_FONTS_FILES,
        .checksum = MMAP_FONTS_CHECKSUM,
        .flags = {
            .mmap_enable = 1,
        },
    };

    esp_err_t ret = mmap_assets_new(&mmap_cfg, &s_font_assets);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to map font assets: %s", esp_err_to_name(ret));
        return ret;
    }

    const int stored_files = mmap_assets_get_stored_files(s_font_assets);
    if (stored_files != MMAP_FONTS_FILES) {
        ESP_LOGE(TAG, "Expected %d fonts, found %d", MMAP_FONTS_FILES, stored_files);
        (void)mmap_assets_del(s_font_assets);
        s_font_assets = NULL;
        return ESP_ERR_INVALID_SIZE;
    }

    const fs_cfg_t fs_cfg = {
        .fs_letter = DASHBOARD_FONT_DRIVE_LETTER,
        .fs_nums = stored_files,
        .fs_assets = s_font_assets,
    };

    ret = esp_lv_fs_desc_init(&fs_cfg, &s_font_fs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount LVGL FS: %s", esp_err_to_name(ret));
        (void)mmap_assets_del(s_font_assets);
        s_font_assets = NULL;
        return ret;
    }

    s_assets_ready = true;
    return ESP_OK;
}

/* LVGL initialises FreeType during lv_init() when LV_USE_FREETYPE is enabled. */
static lv_font_t *create_font(uint16_t size_px, dashboard_font_weight_t weight)
{
    const char *font_path = (weight == DASHBOARD_FONT_WEIGHT_SEMIBOLD)
                                ? "F:AzeretMonoSemiBold.ttf"
                                : "F:AzeretMonoRegular.ttf";

    return lv_freetype_font_create(font_path, LV_FREETYPE_FONT_RENDER_MODE_BITMAP,
                                   size_px, LV_FREETYPE_FONT_STYLE_NORMAL);
}

esp_err_t dashboard_font_get(uint16_t size_px, dashboard_font_weight_t weight, const lv_font_t **font)
{
    if (font == NULL || size_px == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    *font = NULL;

    esp_err_t ret = ensure_assets_mounted();
    if (ret != ESP_OK) {
        return ret;
    }

    for (size_t i = 0; i < s_cache_count; i++) {
        if (s_cache[i].size_px == size_px && s_cache[i].weight == weight) {
            *font = s_cache[i].font;
            return ESP_OK;
        }
    }

    if (s_cache_count >= DASHBOARD_FONT_CACHE_MAX) {
        return ESP_ERR_NO_MEM;
    }

    lv_font_t *created = create_font(size_px, weight);
    if (created == NULL) {
        ESP_LOGE(TAG, "Failed to create font: size=%u weight=%d", (unsigned)size_px, (int)weight);
        return ESP_FAIL;
    }

    s_cache[s_cache_count].size_px = size_px;
    s_cache[s_cache_count].weight = weight;
    s_cache[s_cache_count].font = created;
    s_cache_count++;

    *font = created;
    return ESP_OK;
}
