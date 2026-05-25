/**
 * @file fp_storage.h
 * @brief Layer 4 — LittleFS-based RAW Fingerprint Image Storage.
 *
 * ============================================================
 * BEGINNER EXPLANATION — LITTLEFS AND RAW STORAGE
 * ============================================================
 * LittleFS is a tiny filesystem designed for microcontrollers.
 * It stores files in the ESP32's internal flash memory — the
 * same flash that holds your firmware, just a different region.
 *
 * FLASH MEMORY vs RAM:
 *   RAM (SRAM)  : 320KB, very fast, loses data on power-off
 *   Flash       : 4MB,   slower,   keeps data after power-off
 *
 * We store fingerprint images in Flash via LittleFS so they
 * persist between reboots.
 *
 * RAW FILE FORMAT (.raw):
 *   A .raw file is just raw binary — no headers, no compression,
 *   no metadata. Just the 36,864 bytes of 4-bit packed pixels
 *   exactly as they came out of the sensor.
 *   - File size: always exactly 36,864 bytes
 *   - Pixel order: left-to-right, top-to-bottom
 *   - Each byte: [pixel_N in bits7-4] [pixel_N+1 in bits3-0]
 *
 * STREAMING WRITE PATTERN:
 *   We open the file once, then call fp_storage_write_chunk()
 *   for each 128-byte UART packet — writing directly to flash.
 *   The full image is NEVER stored in RAM. Only 128 bytes live
 *   in RAM at any one time. This is critical for ESP32 with 320KB RAM.
 *
 * FILE STRUCTURE:
 *   /fp/fingerprints/
 *     1/          <- User ID 1
 *       fp1.raw   <- Image 1 of 5 (36,864 bytes)
 *       fp2.raw   <- Image 2 of 5
 *       fp3.raw   <- Image 3 of 5
 *       fp4.raw   <- Image 4 of 5
 *       fp5.raw   <- Image 5 of 5
 *       meta.txt  <- User metadata (key=value text file)
 *     2/          <- User ID 2
 *       ...
 *     tmp/        <- Temporary folder
 *       verify.raw <- Captured during VERIFY, deleted after comparison
 * ============================================================
 */
#ifndef FP_STORAGE_H
#define FP_STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "fp_driver.h"   /* For fp_data_callback_t */

/* ============================================================
 * USER METADATA STRUCTURE
 * ============================================================
 * Stored as plain text in meta.txt so you can read it with
 * any text editor or terminal app for debugging.
 */
typedef struct {
    uint8_t  user_id;                  /* Unique user identifier (1–255)       */
    uint8_t  image_count;              /* How many images were captured         */
    uint8_t  threshold;                /* Per-user similarity threshold %       */
    char     capture_date[16];         /* Date of registration "YYYY-MM-DD"     */
} fp_meta_t;

/* ============================================================
 * PUBLIC API — Initialization
 * ============================================================ */

/**
 * @brief Mount LittleFS and create directory structure.
 *
 * Mounts the LittleFS partition (label "littlefs" from partitions.csv).
 * Creates /fp/fingerprints/ and /fp/fingerprints/tmp/ if they don't exist.
 * Auto-formats the partition on first boot.
 *
 * @return ESP_OK on success
 */
esp_err_t fp_storage_init(void);

/* ============================================================
 * PUBLIC API — Streaming Write (for Registration)
 * ============================================================
 * These three functions work together:
 *   fp_storage_open_write()   ← open the file
 *   fp_storage_write_chunk()  ← called for each UART packet
 *   fp_storage_close_write()  ← flush and close
 *
 * Or use fp_storage_write_callback() as the fp_data_callback_t
 * in fp_upload_image() to do all three automatically.
 */

/**
 * @brief Open a user's fingerprint image file for streaming write.
 * @param user_id   User ID (1–255)
 * @param img_num   Image number (1–FP_IMAGES_PER_USER)
 * @return ESP_OK on success
 */
esp_err_t fp_storage_open_write(uint8_t user_id, uint8_t img_num);

/**
 * @brief Write a chunk of image data to the currently open file.
 * Called by fp_storage_write_callback() for each UART data packet.
 * @param data  Pixel bytes to write
 * @param len   Number of bytes
 * @return ESP_OK on success
 */
esp_err_t fp_storage_write_chunk(const uint8_t *data, size_t len);

/**
 * @brief Flush and close the currently open streaming write file.
 * @return ESP_OK
 */
esp_err_t fp_storage_close_write(void);

/**
 * @brief Callback adaptor: use this as fp_data_callback_t argument to fp_upload_image().
 *
 * EXAMPLE:
 *   fp_storage_open_write(user_id, img_num);
 *   fp_upload_image(fp_storage_write_callback, NULL, &bytes);
 *   fp_storage_close_write();
 *
 * @return 0 on success, -1 on write error (will abort the UART transfer)
 */
int fp_storage_write_callback(const uint8_t *data, size_t len, void *ctx);

/* ============================================================
 * PUBLIC API — Temp File (for Verification)
 * ============================================================ */

/** @brief Open the temporary verification image file for streaming write. */
esp_err_t fp_storage_open_write_tmp(void);

/** @brief Delete the temporary verification image file. */
void fp_storage_delete_tmp(void);

/** @brief Get the full path to the temporary verification image. */
void fp_storage_get_tmp_path(char *buf, size_t buf_size);

/** @brief Get the full path to the temporary verification binary image. */
void fp_storage_get_tmp_bin_path(char *buf, size_t buf_size);

/* ============================================================
 * PUBLIC API — Metadata
 * ============================================================ */

/** @brief Write user metadata to meta.txt. */
esp_err_t fp_storage_write_meta(uint8_t user_id, const fp_meta_t *meta);

/** @brief Read user metadata from meta.txt. */
esp_err_t fp_storage_read_meta(uint8_t user_id, fp_meta_t *meta);

/* ============================================================
 * PUBLIC API — File Path Helpers
 * ============================================================ */

/**
 * @brief Build the full file path to a stored fingerprint image.
 * EXAMPLE: user_id=2, img_num=3 → "/fp/fingerprints/2/fp3.raw"
 */
void fp_storage_get_image_path(uint8_t user_id, uint8_t img_num,
                                char *buf, size_t buf_size);

/**
 * @brief Build the full file path to a stored binary fingerprint image.
 */
void fp_storage_get_bin_path(uint8_t user_id, uint8_t img_num,
                             char *buf, size_t buf_size);

/* ============================================================
 * PUBLIC API — Binary Storage Upgrade
 * ============================================================ */
esp_err_t fp_storage_open_write_bin(uint8_t user_id, uint8_t img_num);
esp_err_t fp_storage_write_chunk_bin(const uint8_t *data, size_t len);
void fp_storage_delete_bin(uint8_t user_id, uint8_t img_num);

/* ============================================================
 * PUBLIC API — User Management
 * ============================================================ */

/** @brief Returns true if user_id is registered (meta.txt exists). */
bool fp_storage_user_exists(uint8_t user_id);

/** @brief Delete all images and meta.txt for a user. */
esp_err_t fp_storage_delete_user(uint8_t user_id);

/** @brief Delete ALL fingerprint data (all users). */
esp_err_t fp_storage_delete_all(void);

/**
 * @brief Get list of all registered user IDs.
 * @param user_ids  Output array to fill with user IDs
 * @param max_users Maximum number of IDs to return
 * @return Number of registered users found
 */
int fp_storage_list_users(uint8_t *user_ids, int max_users);

/**
 * @brief Get LittleFS usage statistics.
 * @param total_bytes  Output: total partition size
 * @param used_bytes   Output: bytes currently used
 * @param user_count   Output: number of registered users
 */
void fp_storage_get_info(size_t *total_bytes, size_t *used_bytes, int *user_count);

#endif /* FP_STORAGE_H */
