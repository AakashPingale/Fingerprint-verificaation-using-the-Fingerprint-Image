/**
 * @file attendance.c
 * @brief Layer 6 — Attendance Manager (System Orchestrator).
 *
 * ============================================================
 * THIS IS THE "TOP LAYER" — THE CONDUCTOR
 * ============================================================
 * Like a conductor in an orchestra, this layer doesn't play
 * any instrument itself. It coordinates all other layers:
 *
 *   bt_serial  → receives commands
 *   cmd_parser → identifies what to do
 *   attendance → (this file) orchestrates the sequence of operations
 *   fp_driver  → captures images from sensor
 *   fp_storage → saves/loads images from flash
 *   fp_compare → computes similarity between images
 *
 * REGISTRATION FLOW (SET,ID:n):
 * ══════════════════════════════
 *   repeat 5 times:
 *     1. Ask user to place finger (via Bluetooth)
 *     2. Try fp_gen_image() up to FP_MAX_CAPTURE_RETRIES times
 *     3. If success: open file → fp_upload_image() → close file
 *     4. Send progress over Bluetooth
 *     5. Ask user to lift and replace finger (except after last)
 *     6. Wait FP_LIFT_DELAY_MS before next capture
 *   Write meta.txt
 *   Send "OK,SET,ID:n,DONE"
 *
 * VERIFICATION FLOW (VERIFY):
 * ════════════════════════════
 *   1. Capture one fingerprint → store in /tmp/verify.raw
 *   2. For each registered user (all of them):
 *      a. Read user's meta.txt (get image count + threshold)
 *      b. Compare /tmp/verify.raw vs each stored image
 *      c. Track best (highest similarity) score and user
 *   3. Delete /tmp/verify.raw
 *   4. If best_sim >= threshold → MATCH response + log event
 *      Else                    → NO_MATCH response
 * ============================================================
 */
#include "attendance.h"
#include "bt_serial.h"
#include "fp_driver.h"
#include "fp_storage.h"
#include "fp_compare.h"
#include "fp_config.h"
#include "fp_debug.h"       /* ASCII art terminal image printing */
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

static const char *TAG = "attendance";

/* ============================================================
 * PRIVATE HELPER: Capture one fingerprint image with retries
 * ============================================================
 * The R307S may return FP_ERR_NO_FINGER if the user hasn't
 * fully placed their finger. We retry several times with a
 * short delay to account for finger placement latency.
 *
 * @return FP_OK when image is captured, FP_ERR_NO_FINGER on exhaustion
 */
static int capture_with_retry(void)
{
    for (int attempt = 0; attempt < FP_MAX_CAPTURE_RETRIES; attempt++) {

        int ret = fp_gen_image();

        if (ret == FP_OK) {
            return FP_OK;
        }

        if (ret == FP_ERR_NO_FINGER) {
            /* Finger not fully placed yet — wait and retry */
            ESP_LOGD(TAG, "No finger, attempt %d/%d",
                     attempt + 1, FP_MAX_CAPTURE_RETRIES);
            vTaskDelay(pdMS_TO_TICKS(FP_RETRY_DELAY_MS));
            continue;
        }

        if (ret == FP_ERR_BAD_IMAGE) {
            /* Image too blurry — ask user to clean finger or press harder */
            bt_send("ERR,BAD_IMAGE — press finger firmly and retry\r\n");
            vTaskDelay(pdMS_TO_TICKS(FP_RETRY_DELAY_MS));
            continue;
        }

        /* Other error (timeout, UART) — report and stop retrying */
        bt_sendf("ERR,SENSOR_ERR:%d\r\n", ret);
        return ret;
    }

    return FP_ERR_NO_FINGER;   /* All retries exhausted */
}

/* ============================================================
 * PRIVATE HELPER: Countdown delay between registrations
 * ============================================================
 * Gives the user time to:
 *   1. Lift their finger off the sensor
 *   2. Re-position their finger (slightly different angle)
 *
 * WHY DIFFERENT ANGLES?
 * Capturing 5 images with slightly different placements gives
 * a better "coverage set" of that user's fingerprint.
 * Rotating/shifting slightly between captures means that during
 * VERIFY, at least one stored image will have a similar
 * orientation to the newly captured finger — giving a high
 * similarity score even if the user places at an angle.
 *
 * WHAT HAPPENS:
 *   [✓ Saved!] → LIFT FINGER → 3... → 2... → 1... → PLACE!
 *
 * @param next_img   The image number coming up next (for display)
 */
static void countdown_before_next(int next_img)
{
    /* ── Phase 1: Signal that the image was saved and ask to LIFT ── */
    bt_send("\r\n>>> Saved! LIFT YOUR FINGER NOW! <<<\r\n");
    ESP_LOGI(TAG, "Waiting for finger lift...");

    /* Give the user enough time to physically lift their finger.
     * The sensor may still detect the finger for ~500ms after lift. */
    vTaskDelay(pdMS_TO_TICKS(1500));

    /* ── Phase 2: Announce what's coming ── */
    bt_sendf("\r\nNext image (%d/%d) in:\r\n", next_img, FP_IMAGES_PER_USER);

    /* ── Phase 3: 3-second countdown with BT and serial messages ── */
    for (int c = 3; c > 0; c--) {
        bt_sendf("  %d...\r\n", c);
        ESP_LOGI(TAG, "Countdown: %d", c);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* ── Phase 4: GO! ── */
    bt_send(">>> PLACE FINGER NOW! <<<\r\n");
    ESP_LOGI(TAG, "Countdown done — waiting for finger placement");

    /* Small grace period: capture_with_retry() will start polling
     * after a 500ms initial pause (see attendance_register_user) */
    vTaskDelay(pdMS_TO_TICKS(300));
}

/* ============================================================
 * attendance_register_user — Handle SET,ID:<n>
 * ============================================================ */
esp_err_t attendance_register_user(uint8_t user_id)
{
    /* Check if the fingerprint sensor is connected */
    if (!fp_is_detected()) {
        /* Try a quick re-detect in case they connected it after boot */
        fp_detect_module();
    }
    
    if (!fp_is_detected()) {
        bt_send("ERR,SENSOR_NOT_DETECTED — The fingerprint module is not responding.\r\n");
        bt_send("Please check wiring: Pin 17 (TX) -> Sensor RX, Pin 16 (RX) -> Sensor TX.\r\n");
        return ESP_FAIL;
    }

    bt_sendf("\r\n=== Registering User %d ===\r\n", user_id);

    bt_sendf("Will capture %d fingerprint images.\r\n", FP_IMAGES_PER_USER);

    /* Capture FP_IMAGES_PER_USER (5) images */
    for (int img = 1; img <= FP_IMAGES_PER_USER; img++) {

        /* ── Tell user what to do ── */
        bt_sendf("\r\nImage %d/%d — Place finger on sensor...\r\n",
                 img, FP_IMAGES_PER_USER);

        /* ── Wait briefly so user reads the message ── */
        vTaskDelay(pdMS_TO_TICKS(500));

        /* ── Capture with retry loop ── */
        int ret = capture_with_retry();
        if (ret != FP_OK) {
            bt_sendf("ERR,NO_FINGER,IMG:%d — Registration aborted\r\n", img);
            ESP_LOGE(TAG, "register_user %d: capture failed for img %d", user_id, img);
            return ESP_FAIL;
        }

        ESP_LOGI(TAG, "register_user %d: img %d captured OK", user_id, img);

        /* ── Open destination file for streaming write ── */
        esp_err_t err = fp_storage_open_write(user_id, (uint8_t)img);
        if (err != ESP_OK) {
            bt_sendf("ERR,FLASH_OPEN,IMG:%d\r\n", img);
            return err;
        }

        /* ── Stream image from sensor to LittleFS ──
         * fp_upload_image calls fp_storage_write_callback for each
         * 128-byte UART packet, which calls fp_storage_write_chunk(),
         * which calls fwrite() directly to the open file.
         * ZERO full-image buffering in RAM. */
        size_t bytes_rx = 0;
        ret = fp_upload_image(fp_storage_write_callback, NULL, &bytes_rx);

        /* ── Always close the file, even on error ── */
        fp_storage_close_write();

        if (ret != FP_OK) {
            bt_sendf("ERR,UPLOAD_FAIL,IMG:%d,ERR:%d\r\n", img, ret);
            ESP_LOGE(TAG, "register_user %d: upload failed for img %d (err=%d)",
                     user_id, img, ret);
            return ESP_FAIL;
        }

        bt_sendf("OK,SET,ID:%d,IMG:%d,BYTES:%zu\r\n", user_id, img, bytes_rx);
        ESP_LOGI(TAG, "register_user %d: img %d saved (%zu bytes)",
                 user_id, img, bytes_rx);

        /* ── AUTO-PRINT: Show the captured image as ASCII art in the serial monitor.
         * This lets you visually confirm the image quality right after capture.
         * Visible in: idf.py monitor
         * Does NOT send image over Bluetooth (too much data for BT SPP). */
        {
            char img_path[FP_MAX_PATH];
            fp_storage_get_image_path(user_id, (uint8_t)img, img_path, sizeof(img_path));
            ESP_LOGI(TAG, "Printing captured image to terminal...");
            fp_debug_print_terminal(img_path, FP_DBG_COL_STEP, FP_DBG_ROW_STEP);
        }

        /* ── Countdown before next image (except after the last one) ── */
        if (img < FP_IMAGES_PER_USER) {
            countdown_before_next(img + 1);
        }
    }

    /* ── Write metadata file ── */
    fp_meta_t meta = {
        .user_id     = user_id,
        .image_count = FP_IMAGES_PER_USER,
        .threshold   = FP_DEFAULT_THRESHOLD,
    };
    /* Simple date string using FreeRTOS uptime (no RTC in this example).
     * If you have an RTC module, replace this with real date/time. */
    uint32_t uptime_sec = (uint32_t)(xTaskGetTickCount() / configTICK_RATE_HZ);
    snprintf(meta.capture_date, sizeof(meta.capture_date),
             "uptime_%lus", (unsigned long)uptime_sec);

    fp_storage_write_meta(user_id, &meta);

    /* ── Done ── */
    bt_sendf("OK,SET,ID:%d,DONE — All %d images stored\r\n",
             user_id, FP_IMAGES_PER_USER);
    bt_sendf("Similarity threshold: %d%%\r\n", FP_DEFAULT_THRESHOLD);

    return ESP_OK;
}

/* ============================================================
 * attendance_delete_user — Handle DEL,ID:<n>
 * ============================================================ */
void attendance_delete_user(uint8_t user_id)
{
    if (!fp_storage_user_exists(user_id)) {
        bt_sendf("ERR,USER_NOT_FOUND,ID:%d\r\n", user_id);
        return;
    }

    esp_err_t err = fp_storage_delete_user(user_id);
    if (err == ESP_OK) {
        bt_sendf("OK,DEL,ID:%d — User deleted\r\n", user_id);
    } else {
        bt_sendf("ERR,DEL_FAILED,ID:%d\r\n", user_id);
    }
}

/* ============================================================
 * attendance_verify — Handle VERIFY command
 * ============================================================ */
void attendance_verify(void)
{
    /* Check if the fingerprint sensor is connected */
    if (!fp_is_detected()) {
        /* Try a quick re-detect in case they connected it after boot */
        fp_detect_module();
    }
    
    if (!fp_is_detected()) {
        bt_send("ERR,SENSOR_NOT_DETECTED — The fingerprint module is not responding.\r\n");
        bt_send("Please check wiring: Pin 17 (TX) -> Sensor RX, Pin 16 (RX) -> Sensor TX.\r\n");
        return;
    }

    bt_send("\r\n=== Fingerprint Verification ===\r\n");

    bt_send("Place finger on sensor...\r\n");

    /* ── STEP 1: Capture one fingerprint image ── */
    int ret = capture_with_retry();
    if (ret != FP_OK) {
        bt_send("ERR,NO_FINGER — Verification failed\r\n");
        return;
    }

    /* ── STEP 2: Store capture to temp file ──
     * We need to store it because we compare it against MULTIPLE users.
     * The sensor's ImageBuffer can only hold one image at a time. */
    esp_err_t err = fp_storage_open_write_tmp();
    if (err != ESP_OK) {
        bt_send("ERR,FLASH_OPEN_TMP\r\n");
        return;
    }

    size_t bytes_rx = 0;
    ret = fp_upload_image(fp_storage_write_callback, NULL, &bytes_rx);
    fp_storage_close_write();  /* Always close */

    if (ret != FP_OK) {
        bt_sendf("ERR,UPLOAD_FAILED:%d\r\n", ret);
        fp_storage_delete_tmp();
        return;
    }

    bt_sendf("Captured %zu bytes. Comparing against registered users...\r\n",
             bytes_rx);

    /* Get path to the temporary verification image */
    char tmp_path[FP_MAX_PATH];
    fp_storage_get_tmp_path(tmp_path, sizeof(tmp_path));

    /* ── STEP 3: Compare against all registered users ── */
    uint8_t user_ids[FP_MAX_USERS];
    int user_count = fp_storage_list_users(user_ids, FP_MAX_USERS);

    if (user_count == 0) {
        fp_storage_delete_tmp();
        bt_send("ERR,NO_USERS — No users registered\r\n");
        return;
    }

    uint8_t best_sim = 0;
    int8_t  best_uid = -1;
    uint8_t best_img_idx = 1;

    for (int u = 0; u < user_count; u++) {
        uint8_t uid = user_ids[u];

        /* Read this user's metadata to know how many images they have */
        fp_meta_t meta;
        if (fp_storage_read_meta(uid, &meta) != ESP_OK) {
            ESP_LOGW(TAG, "verify: cannot read meta for user %d, skipping", uid);
            continue;
        }

        bt_sendf("  Checking user %d (%d images)...\r\n", uid, meta.image_count);

        /* Compare query against all stored images for this user */
        uint8_t user_best = 0;
        uint8_t user_best_idx = 1;
        int cmp_ret = fp_compare_best_for_user(tmp_path, uid,
                                               meta.image_count, &user_best, &user_best_idx);
        if (cmp_ret != FP_OK) {
            bt_sendf("  ERR,COMPARE_FAILED,ID:%d\r\n", uid);
            continue;
        }

        bt_sendf("  User %d: best similarity = %d%%\r\n", uid, user_best);

        if (user_best > best_sim) {
            best_sim = user_best;
            best_uid = (int8_t)uid;
            best_img_idx = user_best_idx;
        }
    }

    /* ── STEP 4: Report result & Show side-by-side comparison in terminal ──
     * Compare best_sim against threshold.
     * We use the global default threshold (FP_DEFAULT_THRESHOLD).
     * For per-user thresholds, read from user's meta.txt. */
    bt_send("\r\n--- Result ---\r\n");

    if (best_uid >= 0 && best_sim >= FP_DEFAULT_THRESHOLD) {
        /* ✅ MATCH */
        bt_sendf("MATCH,ID:%d,SIM:%d%%\r\n", best_uid, best_sim);
        ESP_LOGI(TAG, "VERIFY: MATCH user=%d sim=%d%%", best_uid, best_sim);

        /* Print side-by-side visual comparison in terminal stdout */
        char best_stored_path[FP_MAX_PATH];
        fp_storage_get_image_path((uint8_t)best_uid, best_img_idx, best_stored_path, sizeof(best_stored_path));
        fp_debug_compare_visual(best_stored_path, tmp_path, best_sim);

        attendance_log_event((uint8_t)best_uid, best_sim);
    } else {
        /* ❌ NO MATCH */
        if (best_uid >= 0) {
            bt_sendf("NO_MATCH — Best was user %d at %d%% (threshold: %d%%)\r\n",
                     best_uid, best_sim, FP_DEFAULT_THRESHOLD);

            /* Even on NO_MATCH, visually display the closest matching fingerprint */
            char best_stored_path[FP_MAX_PATH];
            fp_storage_get_image_path((uint8_t)best_uid, best_img_idx, best_stored_path, sizeof(best_stored_path));
            fp_debug_compare_visual(best_stored_path, tmp_path, best_sim);
        } else {
            bt_send("NO_MATCH — No users to compare against\r\n");
        }
        ESP_LOGI(TAG, "VERIFY: NO_MATCH, best_sim=%d%%", best_sim);
    }

    /* ── STEP 5: Clean up temp file ── */
    fp_storage_delete_tmp();
}

/* ============================================================
 * attendance_list — Handle LIST command
 * ============================================================ */
void attendance_list(void)
{
    uint8_t user_ids[FP_MAX_USERS];
    int count = fp_storage_list_users(user_ids, FP_MAX_USERS);

    if (count == 0) {
        bt_send("NO_USERS\r\n");
        return;
    }

    bt_sendf("USERS:%d — ", count);

    /* Build comma-separated ID list */
    char buf[128] = {0};
    int pos = 0;
    for (int i = 0; i < count; i++) {
        pos += snprintf(buf + pos, sizeof(buf) - pos,
                        "%d%s", user_ids[i], (i < count - 1) ? "," : "");
    }
    bt_sendf("[%s]\r\n", buf);

    /* Also show per-user details */
    for (int i = 0; i < count; i++) {
        fp_meta_t meta;
        if (fp_storage_read_meta(user_ids[i], &meta) == ESP_OK) {
            bt_sendf("  ID:%d  images=%d  threshold=%d%%  date=%s\r\n",
                     meta.user_id, meta.image_count,
                     meta.threshold, meta.capture_date);
        }
    }
}

/* ============================================================
 * attendance_info — Handle INFO command
 * ============================================================ */
void attendance_info(void)
{
    size_t total = 0, used = 0;
    int    user_count = 0;

    fp_storage_get_info(&total, &used, &user_count);

    size_t free_bytes = (total > used) ? (total - used) : 0;

    bt_send("\r\n--- System Info ---\r\n");
    
    if (fp_is_detected()) {
        bt_send("Sensor Status: DETECTED & verified (R307S)\r\n");
    } else {
        bt_send("Sensor Status: NOT DETECTED (Check wiring and power!)\r\n");
    }

    bt_sendf("Flash total : %zu KB (%zu bytes)\r\n", total / 1024, total);
    bt_sendf("Flash used  : %zu KB (%zu bytes)\r\n", used  / 1024, used);
    bt_sendf("Flash free  : %zu KB (%zu bytes)\r\n", free_bytes / 1024, free_bytes);
    bt_sendf("Users       : %d / %d max\r\n", user_count, FP_MAX_USERS);
    bt_sendf("Images      : %d (est %d × %d)\r\n",
             user_count * FP_IMAGES_PER_USER,
             user_count, FP_IMAGES_PER_USER);
    bt_sendf("Image size  : %d bytes (%.1f KB)\r\n",
             FP_IMAGE_SIZE, FP_IMAGE_SIZE / 1024.0f);
    bt_sendf("Image fmt   : %dx%d px, 4-bit grayscale, 2px/byte\r\n",
             FP_IMAGE_WIDTH, FP_IMAGE_HEIGHT);

    /* Estimate remaining capacity */
    uint32_t bytes_per_user = (uint32_t)FP_IMAGES_PER_USER * FP_IMAGE_SIZE;
    int remaining_users = (bytes_per_user > 0)
                        ? (int)(free_bytes / bytes_per_user)
                        : 0;
    bt_sendf("Capacity left: ~%d more users\r\n", remaining_users);
}

/* ============================================================
 * attendance_clear — Handle CLEAR command
 * ============================================================ */
void attendance_clear(void)
{
    bt_send("WARNING: Deleting ALL fingerprint data...\r\n");
    bt_send("(This cannot be undone. Clearing in 2 seconds...)\r\n");
    vTaskDelay(pdMS_TO_TICKS(2000));  /* Short pause to allow user to see warning */
    esp_err_t err = fp_storage_delete_all();
    if (err == ESP_OK) {
        bt_send("OK,CLEAR,ALL_DELETED\r\n");
    } else {
        bt_send("ERR,CLEAR_FAILED\r\n");
    }
}

/* ============================================================
 * attendance_log_event — Append a match event to the log file
 * ============================================================
 * Log format (plain text, one event per line):
 *   USER:1,SIM:87%,UPTIME:12345000ms
 *
 * BEGINNER NOTE:
 * We use FreeRTOS uptime (xTaskGetTickCount) because we don't
 * have an RTC (Real-Time Clock) module in this basic design.
 * If you add an RTC (e.g., DS3231), replace the uptime with
 * the real date and time string.
 */
void attendance_log_event(uint8_t user_id, uint8_t similarity)
{
    FILE *f = fopen(FP_ATTEND_LOG, "a");   /* "a" = append mode */
    if (!f) {
        ESP_LOGW(TAG, "Cannot open attendance log: %s", FP_ATTEND_LOG);
        return;
    }

    /* Use FreeRTOS tick count as a simple timestamp */
    uint32_t uptime_ms = (uint32_t)(xTaskGetTickCount()
                                    * portTICK_PERIOD_MS);

    fprintf(f, "USER:%d,SIM:%d%%,UPTIME:%lums\n",
            user_id, similarity, (unsigned long)uptime_ms);
    fclose(f);

    bt_sendf("LOG: Attendance recorded for user %d at %d%% match\r\n",
             user_id, similarity);
    ESP_LOGI(TAG, "Attendance logged: user=%d sim=%d%% uptime=%lums",
             user_id, similarity, (unsigned long)uptime_ms);
}

/* ============================================================
 * attendance_auto_verify_task — Background continuous scan task
 * ============================================================
 * Runs forever. Polls the sensor every 500 ms.
 * When a finger is detected it runs a full verification and
 * prints the result to the ESP-IDF terminal.
 * No Bluetooth command is required.
 * ============================================================ */
void attendance_auto_verify_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Auto-verify task started — sensor always scanning");

    while (true) {

        /* Quick poll: is a finger present? */
        int ret = fp_gen_image();

        if (ret == FP_ERR_NO_FINGER) {
            /* Nothing on the sensor — idle, wait 500 ms then poll again */
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (ret != FP_OK) {
            /* Sensor error — log and wait before retrying */
            ESP_LOGW(TAG, "auto_verify: sensor error %d, retrying in 1s", ret);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        /* ── Finger detected — run a full verification ── */
        ESP_LOGI(TAG, "auto_verify: finger detected, running verification...");
        attendance_verify();

        /* Wait 2 seconds so the same finger doesn't trigger again immediately */
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    vTaskDelete(NULL);   /* Never reached */
}
