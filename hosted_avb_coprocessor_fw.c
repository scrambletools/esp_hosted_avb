/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * Host side: the coprocessor's firmware update over esp-hosted's OTA calls
 * (begin, write, end, activate), for a host that offers it to ATDECC
 * controllers through esp_avb. Nothing runs between updates.
 */
#include "hosted_avb_coprocessor_fw.h"

#include <stdio.h>
#include <string.h>

#include "eh_host_cp_ota.h"
#include "esp_app_desc.h"
#include "esp_app_format.h"
#include "esp_log.h"
#include "sdkconfig.h"

#if CONFIG_ESP_HOSTED_CP_TARGET_ESP32C6
#define COPROCESSOR_CHIP ESP_CHIP_ID_ESP32C6
#elif CONFIG_ESP_HOSTED_CP_TARGET_ESP32C5
#define COPROCESSOR_CHIP ESP_CHIP_ID_ESP32C5
#elif CONFIG_ESP_HOSTED_CP_TARGET_ESP32C61
#define COPROCESSOR_CHIP ESP_CHIP_ID_ESP32C61
#elif CONFIG_ESP_HOSTED_CP_TARGET_ESP32S3
#define COPROCESSOR_CHIP ESP_CHIP_ID_ESP32S3
#else
#error "hosted_avb_coprocessor_fw: add the coprocessor's chip ID"
#endif

#define WRITE_CHUNK 1536 /* esp-hosted's largest OTA write */

static const char *TAG = "hosted_avb_cpfw";

/* The length an ESP app image's header describes: segments, the
 * checksum padded to 16 octets, and the appended SHA-256. 0 when the
 * segments run past what was uploaded. */
static uint32_t described_length(const uint8_t *image, uint32_t length) {
  const esp_image_header_t *header = (const esp_image_header_t *)image;
  uint32_t offset = sizeof(esp_image_header_t);
  for (int i = 0; i < header->segment_count; i++) {
    esp_image_segment_header_t segment;
    if (offset + sizeof(segment) > length)
      return 0;
    memcpy(&segment, image + offset, sizeof(segment));
    offset += sizeof(segment);
    if (segment.data_len > length - offset)
      return 0;
    offset += segment.data_len;
  }
  offset += 1;                   /* checksum */
  offset = (offset + 15) & ~15u; /* padded to 16 octets */
  if (header->hash_appended == 1)
    offset += 32;
  return offset <= length ? offset : 0;
}

static esp_err_t check_image(const uint8_t *image, uint32_t length) {
  const esp_image_header_t *header = (const esp_image_header_t *)image;
  const uint32_t desc_offset =
      sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t);
  if (length < desc_offset + sizeof(esp_app_desc_t) ||
      header->magic != ESP_IMAGE_HEADER_MAGIC) {
    ESP_LOGE(TAG, "Not an ESP app image");
    return ESP_ERR_INVALID_ARG;
  }
  if (header->chip_id != COPROCESSOR_CHIP) {
    ESP_LOGE(TAG, "Image is for chip %u, the coprocessor is %u",
             header->chip_id, COPROCESSOR_CHIP);
    return ESP_ERR_INVALID_ARG;
  }
  esp_app_desc_t desc;
  memcpy(&desc, image + desc_offset, sizeof(desc));
  if (desc.magic_word != ESP_APP_DESC_MAGIC_WORD ||
      strncmp(desc.project_name, CONFIG_HOSTED_AVB_COPROCESSOR_PROJECT,
              sizeof(desc.project_name)) != 0) {
    ESP_LOGE(TAG, "Image is project '%.32s', not '%s'", desc.project_name,
             CONFIG_HOSTED_AVB_COPROCESSOR_PROJECT);
    return ESP_ERR_INVALID_ARG;
  }
  if (described_length(image, length) != length) {
    ESP_LOGE(TAG, "Image length %lu is not what its header describes",
             (unsigned long)length);
    return ESP_ERR_INVALID_SIZE;
  }
  char elf_sha[11];
  for (int i = 0; i < 5; i++)
    snprintf(elf_sha + 2 * i, 3, "%02x", desc.app_elf_sha256[i]);
  elf_sha[9] = '\0'; /* nine hex digits, as firmware versions show it */
  ESP_LOGW(TAG, "Image '%.32s' version %.32s (ELF %s), %lu bytes",
           desc.project_name, desc.version, elf_sha, (unsigned long)length);
  return ESP_OK;
}

esp_err_t hosted_avb_coprocessor_fw_store(const uint8_t *image,
                                          uint32_t length,
                                          void (*progress)(uint16_t permille),
                                          void *context) {
  (void)context;
  esp_err_t err = check_image(image, length);
  if (err != ESP_OK)
    return err;
  progress(10);
  ESP_LOGW(TAG, "Erasing the coprocessor's spare slot");
  err = eh_host_cp_ota_begin();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Coprocessor OTA begin: %s", esp_err_to_name(err));
    return err;
  }
  progress(100);
  for (uint32_t offset = 0; offset < length; offset += WRITE_CHUNK) {
    uint32_t chunk =
        length - offset < WRITE_CHUNK ? length - offset : WRITE_CHUNK;
    err = eh_host_cp_ota_write(image + offset, chunk);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Coprocessor OTA write at %lu: %s", (unsigned long)offset,
               esp_err_to_name(err));
      eh_host_cp_ota_end(); /* closes the slot, which then fails its check */
      return err;
    }
    progress((uint16_t)(100 + (uint64_t)880 * (offset + chunk) / length));
  }
  err = eh_host_cp_ota_end();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Coprocessor refused the image: %s", esp_err_to_name(err));
    return err;
  }
  progress(990);
  ESP_LOGW(TAG, "Image written and checked by the coprocessor");
  return ESP_OK;
}

esp_err_t hosted_avb_coprocessor_fw_activate(void *context) {
  (void)context;
  esp_err_t err = eh_host_cp_ota_activate();
  if (err == ESP_OK)
    ESP_LOGW(TAG, "Coprocessor switched to the new image");
  return err;
}
