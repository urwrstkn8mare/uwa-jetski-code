#include "ws_display.h"

#include <stdbool.h>
#include <stdint.h>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_lv_adapter.h"
#include "esp_lv_fs.h"
#include "esp_mmap_assets.h"
#include "mmap_generate_fonts.h"
#include "ws_display_internal.h"

static const char *TAG = "ws_display_font";

#define WS_DISPLAY_FONT_PARTITION_LABEL "fonts"
#define WS_DISPLAY_FONT_DRIVE_LETTER 'F'

static mmap_assets_handle_t s_font_assets;
static esp_lv_fs_handle_t s_font_fs;
static bool s_font_assets_ready;

static void release_font_assets(void) {
  if (s_font_fs != NULL) {
    (void)esp_lv_fs_desc_deinit(s_font_fs);
    s_font_fs = NULL;
  }
  if (s_font_assets != NULL) {
    (void)mmap_assets_del(s_font_assets);
    s_font_assets = NULL;
  }

  s_font_assets_ready = false;
}

static esp_err_t mount_font_assets(void) {
  if (s_font_assets_ready) {
    return ESP_OK;
  }

  const mmap_assets_config_t mmap_cfg = {
      .partition_label = WS_DISPLAY_FONT_PARTITION_LABEL,
      .max_files = MMAP_FONTS_FILES,
      .checksum = MMAP_FONTS_CHECKSUM,
      .flags =
          {
              .mmap_enable = 1,
          },
  };

  esp_err_t ret = mmap_assets_new(&mmap_cfg, &s_font_assets);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to map font assets");
    return ret;
  }

  int stored_files = mmap_assets_get_stored_files(s_font_assets);
  if (stored_files <= 0) {
    ret = ESP_ERR_NOT_FOUND;
    ESP_LOGE(TAG, "Font asset is missing");
    goto fail;
  }

  if (stored_files != MMAP_FONTS_FILES) {
    ret = ESP_ERR_INVALID_SIZE;
    ESP_LOGE(TAG, "Expected %d font file(s), found %d", MMAP_FONTS_FILES,
             stored_files);
    goto fail;
  }

  const fs_cfg_t fs_cfg = {
      .fs_letter = WS_DISPLAY_FONT_DRIVE_LETTER,
      .fs_nums = stored_files,
      .fs_assets = s_font_assets,
  };

  ret = esp_lv_fs_desc_init(&fs_cfg, &s_font_fs);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to mount font filesystem");
    goto fail;
  }

  s_font_assets_ready = true;

  ESP_LOGI(TAG, "Mounted font assets partition '%s'",
           WS_DISPLAY_FONT_PARTITION_LABEL);
  return ESP_OK;

fail:
  if (s_font_fs != NULL) {
    (void)esp_lv_fs_desc_deinit(s_font_fs);
    s_font_fs = NULL;
  }

  if (s_font_assets != NULL) {
    (void)mmap_assets_del(s_font_assets);
    s_font_assets = NULL;
  }

  return ret;
}

void ws_display_font_reset(void) { release_font_assets(); }

static const char *weight_to_string(ws_display_font_weight_t weight) {
  return (weight == WS_DISPLAY_FONT_WEIGHT_SEMIBOLD) ? "semibold" : "regular";
}

static esp_err_t create_font_variant(uint16_t size_px,
                                     ws_display_font_weight_t weight,
                                     const lv_font_t **font) {
  ESP_RETURN_ON_FALSE(font != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "Font output must not be NULL");

  *font = NULL;

  ESP_RETURN_ON_ERROR(mount_font_assets(), TAG, "Failed to mount font assets");

  const char *font_path = (weight == WS_DISPLAY_FONT_WEIGHT_SEMIBOLD)
                              ? "F:AzeretMonoSemiBold.ttf"
                              : "F:AzeretMonoRegular.ttf";

  const esp_lv_adapter_ft_font_config_t cfg =
      ESP_LV_ADAPTER_FT_FONT_FILE_CONFIG(
          font_path, size_px,
          (weight == WS_DISPLAY_FONT_WEIGHT_SEMIBOLD)
              ? ESP_LV_ADAPTER_FT_FONT_STYLE_BOLD
              : ESP_LV_ADAPTER_FT_FONT_STYLE_NORMAL);

  esp_err_t ret = ESP_OK;
  esp_lv_adapter_ft_font_handle_t handle = NULL;
  ESP_GOTO_ON_ERROR(esp_lv_adapter_ft_font_init(&cfg, &handle), err, TAG,
                    "Failed to create font variant %u %s", (unsigned)size_px,
                    weight_to_string(weight));

  const lv_font_t *created_font = esp_lv_adapter_ft_font_get(handle);
  ESP_GOTO_ON_FALSE(created_font != NULL, ESP_ERR_NOT_FOUND, err, TAG,
                    "Font handle returned no font for %u %s", (unsigned)size_px,
                    weight_to_string(weight));

  *font = created_font;
  ESP_LOGI(TAG, "Created font variant %u %s", (unsigned)size_px,
           weight_to_string(weight));

  return ESP_OK;

err:
  if (handle != NULL) {
    (void)esp_lv_adapter_ft_font_deinit(handle);
  }
  return ret;
}

esp_err_t ws_display_font_get(uint16_t size_px, ws_display_font_weight_t weight,
                              const lv_font_t **font) {

  ESP_RETURN_ON_FALSE(font != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "Font output must not be NULL");

  ESP_RETURN_ON_FALSE(size_px > 0, ESP_ERR_INVALID_ARG, TAG,
                      "Font size must be greater than 0");
  ESP_RETURN_ON_FALSE(ws_display_lock_held(), ESP_ERR_INVALID_STATE, TAG,
                      "Call ws_display_lock() before loading fonts");
  ESP_RETURN_ON_FALSE(
      esp_lv_adapter_is_initialized(), ESP_ERR_INVALID_STATE, TAG,
      "Display adapter must be initialised before loading fonts");

  return create_font_variant(size_px, weight, font);
}
