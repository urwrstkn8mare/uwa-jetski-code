#include "dashboard_font.h"

#include "esp_log.h"
#include "esp_lv_fs.h"
#include "esp_mmap_assets.h"
#include "mmap_generate_fonts.h"

#if __has_include("src/libs/freetype/lv_freetype.h")
#include "src/libs/freetype/lv_freetype.h"
#include "src/libs/freetype/lv_freetype_private.h"
#elif __has_include("src/extra/libs/freetype/lv_freetype.h")
#include "src/extra/libs/freetype/lv_freetype.h"
#include "src/extra/libs/freetype/lv_freetype_private.h"
#else
#error "LVGL FreeType header not found"
#endif

#include "ft2build.h"
#include FT_MULTIPLE_MASTERS_H

#define DASHBOARD_FONT_DRIVE_LETTER 'F'
#define DASHBOARD_FONT_PARTITION_LABEL "fonts"
#define DASHBOARD_FONT_CACHE_MAX 16

static const char *TAG = "dashboard_font";

typedef struct {
    uint16_t size_px;
    int weight;
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

static esp_err_t apply_variable_weight(lv_font_t *font, uint16_t size_px, int weight)
{
    if (font == NULL || font->dsc == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    lv_freetype_font_dsc_t *font_dsc = (lv_freetype_font_dsc_t *)font->dsc;
    FT_Face face = font_dsc->cache_node ? font_dsc->cache_node->face : NULL;
    if (face == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    FT_MM_Var *mm_var = NULL;
    FT_Error ft_err = FT_Get_MM_Var(face, &mm_var);
    if (ft_err != 0 || mm_var == NULL) {
        ESP_LOGW(TAG, "Variable font axes unavailable for weight %d", weight);
        return ESP_ERR_NOT_SUPPORTED;
    }

    FT_Fixed coords[mm_var->num_axis];
    bool found_weight_axis = false;

    for (FT_UInt i = 0; i < mm_var->num_axis; i++) {
        coords[i] = mm_var->axis[i].def;
        if (mm_var->axis[i].tag == FT_MAKE_TAG('w', 'g', 'h', 't')) {
            FT_Fixed clamped_weight = (FT_Fixed)weight << 16;
            if (clamped_weight < mm_var->axis[i].minimum) {
                clamped_weight = mm_var->axis[i].minimum;
            }
            if (clamped_weight > mm_var->axis[i].maximum) {
                clamped_weight = mm_var->axis[i].maximum;
            }
            coords[i] = clamped_weight;
            found_weight_axis = true;
        }
    }

    if (!found_weight_axis) {
        FT_Done_MM_Var(font_dsc->context->library, mm_var);
        ESP_LOGW(TAG, "Variable font has no wght axis");
        return ESP_ERR_NOT_SUPPORTED;
    }

    ft_err = FT_Set_Var_Design_Coordinates(face, mm_var->num_axis, coords);
    FT_Done_MM_Var(font_dsc->context->library, mm_var);
    if (ft_err != 0) {
        ESP_LOGE(TAG, "Failed to set variable font weight %d (0x%x)", weight, (unsigned)ft_err);
        return ESP_FAIL;
    }

    ft_err = FT_Set_Pixel_Sizes(face, 0, size_px);
    if (ft_err != 0) {
        ESP_LOGE(TAG, "Failed to refresh variable font metrics for %u px (0x%x)",
                 (unsigned)size_px, (unsigned)ft_err);
        return ESP_FAIL;
    }

    font->line_height = FT_F26DOT6_TO_INT(face->size->metrics.height);
    font->base_line = -FT_F26DOT6_TO_INT(face->size->metrics.descender);

    FT_Fixed scale = face->size->metrics.y_scale;
    int8_t thickness = FT_F26DOT6_TO_INT(FT_MulFix(scale, face->underline_thickness));
    font->underline_position = FT_F26DOT6_TO_INT(FT_MulFix(scale, face->underline_position));
    font->underline_thickness = thickness < 1 ? 1 : thickness;

    return ESP_OK;
}

/* LVGL initialises FreeType during lv_init() when LV_USE_FREETYPE is enabled. */
static lv_font_t *create_font(uint16_t size_px, int weight)
{
    static const char *font_path = "F:AzeretMonoVar.ttf";

    lv_font_t *font = lv_freetype_font_create(font_path,
                                              LV_FREETYPE_FONT_RENDER_MODE_BITMAP,
                                              size_px,
                                              LV_FREETYPE_FONT_STYLE_NORMAL);
    if (font == NULL) {
        return NULL;
    }

    if (apply_variable_weight(font, size_px, weight) != ESP_OK) {
        lv_freetype_font_delete(font);
        return NULL;
    }

    return font;
}

esp_err_t dashboard_font_get(uint16_t size_px, int weight, const lv_font_t **font)
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
