/**
 * @file FP_Module.c
 * @brief Application entry point — ESP32 RAW Fingerprint Attendance System.
 *
 * ============================================================
 * SYSTEM OVERVIEW FOR BEGINNERS
 * ============================================================
 * This firmware turns an ESP32 into a fingerprint attendance
 * controller. Here is the big picture:
 *
 *   [Fingerprint Sensor R307S]
 *          │ UART (57600 baud)
 *          ▼
 *   [ESP32 — This Firmware]
 *          │
 *          ├─── Reads RAW fingerprint images (36KB each)
 *          ├─── Stores images in LittleFS flash (5 per user)
 *          ├─── Compares images using pixel-level SAD algorithm
 *          ├─── Returns similarity % and MATCH/NO_MATCH result
 *          └─── Logs attendance events to flash
 *          │
 *          │ Bluetooth Classic SPP
 *          ▼
 *   [Phone/PC — Serial Bluetooth Terminal App]
 *          │
 *          ├─── Sends: SET,ID:1  →  Register user 1
 *          ├─── Sends: VERIFY    →  Identify finger
 *          ├─── Sends: LIST      →  Show all users
 *          └─── Receives results in real-time
 *
 * WHY APP_MAIN IS THIN:
 * =====================
 * app_main() only initialises hardware and starts one task.
 * All logic lives in the layer-specific .c files. This is the
 * "separation of concerns" principle: each file has ONE job.
 *
 * FreeRTOS TASK:
 * ==============
 * The main BT command loop runs in a FreeRTOS task (fp_main_task).
 * FreeRTOS is a real-time operating system that lets you run
 * multiple "tasks" (like threads) on one microcontroller.
 * ============================================================
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_err.h"

/* Our custom component headers */
#include "fp_config.h"
#include "bt_serial.h"
#include "cmd_parser.h"
#include "fp_driver.h"
#include "fp_storage.h"
#include "attendance.h"

static const char *TAG = "FP_Module";

/* ============================================================
 * fp_main_task — The main command processing loop
 * ============================================================
 * This task runs forever, doing:
 *   1. Wait for a Bluetooth command (blocks until one arrives)
 *   2. Process the command (parse → execute → send response)
 *   3. Go back to step 1
 *
 * BLOCKING I/O:
 * bt_wait_command() blocks (yields CPU to other tasks) until
 * a complete command arrives. No CPU is wasted while waiting.
 *
 * @param arg  Not used (required by FreeRTOS task signature)
 */
static void fp_main_task(void *arg)
{
    ESP_LOGI(TAG, "FP main task started. Waiting for Bluetooth commands...");

    char cmd_buf[BT_CMD_MAX_LEN];

    while (true) {
        /* Block here until a complete command line arrives.
         * portMAX_DELAY = wait forever (don't time out). */
        bool got_cmd = bt_wait_command(cmd_buf, sizeof(cmd_buf),
                                       portMAX_DELAY);

        if (got_cmd && cmd_buf[0] != '\0') {
            /* Log what we received (visible in ESP-IDF monitor) */
            ESP_LOGI(TAG, "BT CMD: '%s'", cmd_buf);

            /* Parse and dispatch — calls the appropriate attendance function */
            cmd_process(cmd_buf);
        }
    }

    /* If we somehow exit the loop (we shouldn't), delete this task */
    vTaskDelete(NULL);
}

/* ============================================================
 * app_main — Firmware entry point (called by ESP-IDF on boot)
 * ============================================================
 * BOOT SEQUENCE:
 *   1. NVS init        — required by Bluetooth for pairing info
 *   2. Bluetooth init  — start BT Classic SPP server
 *   3. FP driver init  — configure UART2 for R307S sensor
 *   4. Storage init    — mount LittleFS, prepare directories
 *   5. Start main task — begin processing BT commands
 */
void app_main(void)
{
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, " FP_Module — RAW Fingerprint Attendance");
    ESP_LOGI(TAG, " ESP-IDF RAW Image Biometric System v1.0");
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, " Sensor    : R307S (256x288, 4-bit, 57600 baud)");
    ESP_LOGI(TAG, " BT Name   : %s", BT_DEVICE_NAME);
    ESP_LOGI(TAG, " Threshold : %d%%", FP_DEFAULT_THRESHOLD);
    ESP_LOGI(TAG, " Images/User: %d", FP_IMAGES_PER_USER);
    ESP_LOGI(TAG, "============================================");

    /* ─── STEP 1: Initialize NVS (Non-Volatile Storage) ───
     * NVS is required by the Bluetooth stack to store pairing keys.
     * If NVS is full or has a version mismatch, erase and reinitialize. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS issue (%s) — erasing and reinitializing",
                 esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "NVS init OK");

    /* ─── STEP 2: Initialize Bluetooth Classic SPP ─── */
    err = bt_serial_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BT init FAILED: %s — halting", esp_err_to_name(err));
        /* ESP_ERROR_CHECK would also halt here, but explicit log is clearer */
        return;
    }
    ESP_LOGI(TAG, "BT init OK — device '%s' is discoverable", BT_DEVICE_NAME);

    /* ─── STEP 3: Initialize UART for R307S sensor ─── */
    err = fp_driver_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FP driver init FAILED: %s", esp_err_to_name(err));
        return;
    }
    
    if (fp_is_detected()) {
        ESP_LOGI(TAG, "FP driver init OK — R307S sensor successfully detected and verified!");
    } else {
        ESP_LOGW(TAG, "FP driver init OK — R307S sensor NOT detected (handshake failed)");
        ESP_LOGW(TAG, "************************************************************");
        ESP_LOGW(TAG, "* DIAGNOSTIC WARNING: FINGERPRINT MODULE NOT RESPONDING     *");
        ESP_LOGW(TAG, "* Please double-check your hardware setup:                 *");
        ESP_LOGW(TAG, "*  1. WIRING:                                              *");
        ESP_LOGW(TAG, "*     - ESP32 Pin 17 (TX) -> Fingerprint Module RX         *");
        ESP_LOGW(TAG, "*     - ESP32 Pin 16 (RX) -> Fingerprint Module TX         *");
        ESP_LOGW(TAG, "*  2. POWER:                                               *");
        ESP_LOGW(TAG, "*     - Connect VCC to 3.3V or 5V, and GND to common GND   *");
        ESP_LOGW(TAG, "*  3. BAUD RATE:                                           *");
        ESP_LOGW(TAG, "*     - Sensor must be configured at 57600 baud            *");
        ESP_LOGW(TAG, "************************************************************");
    }


    /* ─── STEP 4: Mount LittleFS and prepare directories ─── */
    err = fp_storage_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Storage init FAILED: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Storage init OK");

    /* ─── STEP 5: Launch main command processing task ───
     * xTaskCreate() creates a FreeRTOS task and starts it immediately.
     * Parameters:
     *   Task function    : fp_main_task
     *   Task name        : "fp_main" (for debugging)
     *   Stack size bytes : MAIN_TASK_STACK_SIZE (8192 from fp_config.h)
     *   Task parameter   : NULL (not used)
     *   Priority         : MAIN_TASK_PRIORITY (5)
     *   Task handle      : NULL (we don't need to reference this task) */
    BaseType_t task_ret = xTaskCreate(fp_main_task,
                                      "fp_main",
                                      MAIN_TASK_STACK_SIZE,
                                      NULL,
                                      MAIN_TASK_PRIORITY,
                                      NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create main task — out of memory?");
        return;
    }

    /* ─── STEP 6: Launch background auto-verify task ───
     * This task continuously polls the sensor and verifies any finger
     * placed on it — no Bluetooth command needed. Runs at lower
     * priority than the BT command task so enrollment still works. */
    task_ret = xTaskCreate(attendance_auto_verify_task,
                           "fp_auto_verify",
                           8192,
                           NULL,
                           4,      /* Priority 4 — one below BT task (5) */
                           NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create auto-verify task — out of memory?");
    }

    ESP_LOGI(TAG, "System ready. Connect via Bluetooth: '%s'", BT_DEVICE_NAME);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Available commands:");
    ESP_LOGI(TAG, "  SET,ID:<n>  — Register user n (5 fingerprints)");
    ESP_LOGI(TAG, "  DEL,ID:<n>  — Delete user n");
    ESP_LOGI(TAG, "  VERIFY      — Identify a finger");
    ESP_LOGI(TAG, "  LIST        — List registered users");
    ESP_LOGI(TAG, "  INFO        — Show flash/storage statistics");
    ESP_LOGI(TAG, "  CLEAR       — Delete ALL data");

    /* app_main() can now return — FreeRTOS scheduler takes over.
     * The fp_main_task continues running in the background. */
}
