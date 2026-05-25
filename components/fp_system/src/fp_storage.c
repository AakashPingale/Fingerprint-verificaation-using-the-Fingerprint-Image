/**
 * @file fp_storage.c
 * @brief Layer 4 — LittleFS RAW Fingerprint Image Storage Implementation.
 *
 * ============================================================
 * HOW LITTLEFS WORKS ON ESP32
 * ============================================================
 * LittleFS uses a region of flash memory (our "littlefs" partition)
 * as a filesystem with:
 *   - Files and directories (like any filesystem)
 *   - Power-loss resilience (incomplete writes don't corrupt data)
 *   - Wear leveling (spreads writes to avoid wearing out one area)
 *
 * We access it using the ESP-IDF VFS (Virtual File System) layer.
 * VFS maps LittleFS files to standard C POSIX file functions:
 *   fopen(), fread(), fwrite(), fclose(), mkdir(), opendir(), etc.
 *
 * STREAMING WRITE DESIGN:
 * ========================
 * Traditional approach (WRONG for ESP32):
 *   1. Allocate uint8_t image_buf[36864]  ← uses 36KB of 320KB total RAM
 *   2. Fill entire buffer from UART
 *   3. Write buffer to file
 *   PROBLEM: We can't afford 36KB + code + stack + BT buffers
 *
 * Our streaming approach (CORRECT):
 *   1. Open the file (s_write_file stays open)
 *   2. For each 128-byte UART packet:
 *      - Receive 128 bytes → fwrite(128 bytes) → immediately written
 *   3. Close file after last packet
 *   RAM used: just 128 bytes at a time ✅
 * ============================================================
 */
#include "fp_storage.h"
#include "fp_config.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>

static const char *TAG = "fp_storage";

/* Currently open file for streaming write operations.
 * NULL when no write is in progress. */
static FILE *s_write_file = NULL;

/* ============================================================
 * fp_storage_init — Mount LittleFS, prepare directory structure
 * ============================================================ */
esp_err_t fp_storage_init(void)
{
    ESP_LOGI(TAG, "Mounting LittleFS (partition='%s', mount='%s')...",
             FP_FS_PARTITION, FP_FS_MOUNT_POINT);

    /* Configuration for LittleFS mount.
     * format_if_mount_failed=true: On very first boot (blank flash),
     * the partition is automatically formatted.  */
    esp_vfs_littlefs_conf_t conf = {
        .base_path             = FP_FS_MOUNT_POINT,
        .partition_label       = FP_FS_PARTITION,
        .format_if_mount_failed = true,
        .dont_mount            = false,
    };

    esp_err_t err = esp_vfs_littlefs_register(&conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LittleFS mount failed: %s", esp_err_to_name(err));
        if (err == ESP_FAIL) {
            ESP_LOGE(TAG, "Hint: try erasing flash with 'idf.py erase-flash'");
        }
        return err;
    }

    /* Log partition size */
    size_t total = 0, used = 0;
    esp_littlefs_info(FP_FS_PARTITION, &total, &used);
    ESP_LOGI(TAG, "LittleFS OK: %zu KB total, %zu KB used, %zu KB free",
             total / 1024, used / 1024, (total - used) / 1024);

    /* Create root directory — mkdir() returns 0 on success, -1 if exists (OK) */
    mkdir(FP_ROOT_DIR,  0777);  /* /fp/fingerprints       */
    mkdir(FP_TMP_DIR,   0777);  /* /fp/fingerprints/tmp   */

    ESP_LOGI(TAG, "Storage ready. Root: %s", FP_ROOT_DIR);
    return ESP_OK;
}

/* ============================================================
 * fp_storage_open_write — Open image file for streaming write
 * ============================================================ */
esp_err_t fp_storage_open_write(uint8_t user_id, uint8_t img_num)
{
    /* Close any previously open file (safety measure) */
    if (s_write_file) {
        ESP_LOGW(TAG, "open_write: closing leaked file handle");
        fclose(s_write_file);
        s_write_file = NULL;
    }

    /* Create user directory first */
    char dir_path[FP_MAX_PATH];
    snprintf(dir_path, sizeof(dir_path), "%s/%d", FP_ROOT_DIR, user_id);
    mkdir(dir_path, 0777);  /* OK if already exists */

    /* Build file path */
    char file_path[FP_MAX_PATH];
    snprintf(file_path, sizeof(file_path), "%s/%d/fp%d.raw",
             FP_ROOT_DIR, user_id, img_num);

    /* Open for binary write — creates file if not exists, truncates if exists */
    s_write_file = fopen(file_path, "wb");
    if (!s_write_file) {
        ESP_LOGE(TAG, "open_write: cannot open '%s': %s",
                 file_path, strerror(errno));
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Opened for write: %s", file_path);
    return ESP_OK;
}

/* ============================================================
 * fp_storage_write_chunk — Write a pixel data chunk to open file
 * ============================================================ */
esp_err_t fp_storage_write_chunk(const uint8_t *data, size_t len)
{
    if (!s_write_file) {
        ESP_LOGE(TAG, "write_chunk: no file open — call open_write first");
        return ESP_ERR_INVALID_STATE;
    }

    size_t written = fwrite(data, 1, len, s_write_file);
    if (written != len) {
        ESP_LOGE(TAG, "write_chunk: wrote %zu of %zu bytes "
                 "(flash full or I/O error)", written, len);
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

/* ============================================================
 * fp_storage_close_write — Flush and close the streaming file
 * ============================================================ */
esp_err_t fp_storage_close_write(void)
{
    if (!s_write_file) {
        return ESP_OK;  /* Nothing to close */
    }

    /* fflush ensures all buffered data is written to flash before fclose */
    fflush(s_write_file);
    fclose(s_write_file);
    s_write_file = NULL;

    ESP_LOGD(TAG, "Write file closed");
    return ESP_OK;
}

/* ============================================================
 * fp_storage_write_callback — Adaptor for fp_data_callback_t
 * ============================================================
 * USAGE: Pass this function directly to fp_upload_image():
 *   fp_storage_open_write(user_id, img_num);
 *   fp_upload_image(fp_storage_write_callback, NULL, &bytes);
 *   fp_storage_close_write();
 */
int fp_storage_write_callback(const uint8_t *data, size_t len, void *ctx)
{
    (void)ctx;  /* Context not needed — using module-level s_write_file */
    esp_err_t err = fp_storage_write_chunk(data, len);
    return (err == ESP_OK) ? 0 : -1;
}

/* ============================================================
 * Temp file functions (for VERIFY command)
 * ============================================================ */

void fp_storage_get_tmp_path(char *buf, size_t buf_size)
{
    snprintf(buf, buf_size, "%s/verify.raw", FP_TMP_DIR);
}

esp_err_t fp_storage_open_write_tmp(void)
{
    if (s_write_file) {
        fclose(s_write_file);
        s_write_file = NULL;
    }

    char path[FP_MAX_PATH];
    fp_storage_get_tmp_path(path, sizeof(path));

    s_write_file = fopen(path, "wb");
    if (!s_write_file) {
        ESP_LOGE(TAG, "open_write_tmp: cannot open '%s': %s",
                 path, strerror(errno));
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Temp verify file opened: %s", path);
    return ESP_OK;
}

void fp_storage_delete_tmp(void)
{
    char path[FP_MAX_PATH];
    fp_storage_get_tmp_path(path, sizeof(path));
    remove(path);
    ESP_LOGD(TAG, "Temp file deleted");
}

/* ============================================================
 * fp_storage_write_meta — Write user metadata to meta.txt
 * ============================================================
 * meta.txt is a plain key=value text file. Example:
 *   user_id=1
 *   image_count=5
 *   image_width=256
 *   image_height=288
 *   image_size=36864
 *   bpp=4
 *   threshold=70
 *   capture_date=2026-05-22
 */
esp_err_t fp_storage_write_meta(uint8_t user_id, const fp_meta_t *meta)
{
    char path[FP_MAX_PATH];
    snprintf(path, sizeof(path), "%s/%d/meta.txt", FP_ROOT_DIR, user_id);

    FILE *f = fopen(path, "w");
    if (!f) {
        ESP_LOGE(TAG, "write_meta: cannot write '%s'", path);
        return ESP_ERR_NO_MEM;
    }

    fprintf(f, "user_id=%d\n",       meta->user_id);
    fprintf(f, "image_count=%d\n",   meta->image_count);
    fprintf(f, "image_width=%d\n",   FP_IMAGE_WIDTH);
    fprintf(f, "image_height=%d\n",  FP_IMAGE_HEIGHT);
    fprintf(f, "image_size=%d\n",    FP_IMAGE_SIZE);
    fprintf(f, "bpp=4\n");
    fprintf(f, "threshold=%d\n",     meta->threshold);
    fprintf(f, "capture_date=%s\n",  meta->capture_date[0]
                                         ? meta->capture_date : "unknown");
    fclose(f);

    ESP_LOGI(TAG, "Meta written: user=%d, imgs=%d, threshold=%d%%",
             meta->user_id, meta->image_count, meta->threshold);
    return ESP_OK;
}

/* ============================================================
 * fp_storage_read_meta — Read user metadata from meta.txt
 * ============================================================ */
esp_err_t fp_storage_read_meta(uint8_t user_id, fp_meta_t *meta)
{
    char path[FP_MAX_PATH];
    snprintf(path, sizeof(path), "%s/%d/meta.txt", FP_ROOT_DIR, user_id);

    FILE *f = fopen(path, "r");
    if (!f) {
        return ESP_ERR_NOT_FOUND;
    }

    /* Set safe defaults before parsing */
    memset(meta, 0, sizeof(*meta));
    meta->user_id     = user_id;
    meta->image_count = FP_IMAGES_PER_USER;
    meta->threshold   = FP_DEFAULT_THRESHOLD;

    char line[64];
    while (fgets(line, sizeof(line), f)) {
        char key[32], value[32];
        /* sscanf parses "key=value" format */
        if (sscanf(line, "%31[^=]=%31s", key, value) == 2) {
            if      (strcmp(key, "user_id")      == 0) meta->user_id     = (uint8_t)atoi(value);
            else if (strcmp(key, "image_count")  == 0) meta->image_count = (uint8_t)atoi(value);
            else if (strcmp(key, "threshold")    == 0) meta->threshold   = (uint8_t)atoi(value);
            else if (strcmp(key, "capture_date") == 0) {
                strncpy(meta->capture_date, value,
                        sizeof(meta->capture_date) - 1);
            }
        }
    }
    fclose(f);
    return ESP_OK;
}

/* ============================================================
 * fp_storage_get_image_path — Build path to a stored image file
 * ============================================================ */
void fp_storage_get_image_path(uint8_t user_id, uint8_t img_num,
                                char *buf, size_t buf_size)
{
    snprintf(buf, buf_size, "%s/%d/fp%d.raw",
             FP_ROOT_DIR, user_id, img_num);
}

/* ============================================================
 * fp_storage_user_exists — Check if a user is registered
 * ============================================================ */
bool fp_storage_user_exists(uint8_t user_id)
{
    char path[FP_MAX_PATH];
    snprintf(path, sizeof(path), "%s/%d/meta.txt", FP_ROOT_DIR, user_id);
    FILE *f = fopen(path, "r");
    if (f) { fclose(f); return true; }
    return false;
}

/* ============================================================
 * fp_storage_delete_user — Remove all files for one user
 * ============================================================ */
esp_err_t fp_storage_delete_user(uint8_t user_id)
{
    char path[FP_MAX_PATH];

    /* Delete each raw image file */
    for (int i = 1; i <= FP_IMAGES_PER_USER; i++) {
        snprintf(path, sizeof(path), "%s/%d/fp%d.raw",
                 FP_ROOT_DIR, user_id, i);
        remove(path);  /* Silently ignore if file doesn't exist */
    }

    /* Delete meta.txt */
    snprintf(path, sizeof(path), "%s/%d/meta.txt", FP_ROOT_DIR, user_id);
    remove(path);

    /* Remove the now-empty directory */
    snprintf(path, sizeof(path), "%s/%d", FP_ROOT_DIR, user_id);
    rmdir(path);  /* Fails if directory not empty — that's OK */

    ESP_LOGI(TAG, "Deleted user %d data", user_id);
    return ESP_OK;
}

/* ============================================================
 * fp_storage_delete_all — Wipe all fingerprint data
 * ============================================================ */
esp_err_t fp_storage_delete_all(void)
{
    DIR *dir = opendir(FP_ROOT_DIR);
    if (!dir) {
        ESP_LOGW(TAG, "delete_all: cannot open root dir");
        return ESP_ERR_NOT_FOUND;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        /* Skip hidden entries and the tmp directory */
        if (entry->d_name[0] == '.') continue;
        if (strcmp(entry->d_name, "tmp") == 0) continue;

        /* Only process numeric directory names (user IDs) */
        int uid = atoi(entry->d_name);
        if (uid > 0) {
            fp_storage_delete_user((uint8_t)uid);
        }
    }
    closedir(dir);

    ESP_LOGI(TAG, "All fingerprint data deleted");
    return ESP_OK;
}

/* ============================================================
 * fp_storage_list_users — Get array of registered user IDs
 * ============================================================ */
int fp_storage_list_users(uint8_t *user_ids, int max_users)
{
    DIR *dir = opendir(FP_ROOT_DIR);
    if (!dir) return 0;

    int count = 0;
    struct dirent *entry;

    while ((entry = readdir(dir)) != NULL && count < max_users) {
        if (entry->d_name[0] == '.') continue;
        if (strcmp(entry->d_name, "tmp") == 0) continue;

        int uid = atoi(entry->d_name);
        if (uid > 0) {
            user_ids[count++] = (uint8_t)uid;
        }
    }
    closedir(dir);
    return count;
}

/* ============================================================
 * fp_storage_get_info — Query filesystem statistics
 * ============================================================ */
void fp_storage_get_info(size_t *total_bytes, size_t *used_bytes, int *user_count)
{
    if (total_bytes && used_bytes) {
        esp_littlefs_info(FP_FS_PARTITION, total_bytes, used_bytes);
    }
    if (user_count) {
        uint8_t ids[FP_MAX_USERS];
        *user_count = fp_storage_list_users(ids, FP_MAX_USERS);
    }
}
