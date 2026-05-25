/**
 * @file attendance.h
 * @brief Layer 6 — Attendance Manager (System Orchestrator).
 *
 * ============================================================
 * BEGINNER EXPLANATION — THE TOP LAYER
 * ============================================================
 * This is the highest-level layer. It orchestrates all other layers
 * to implement the complete attendance system behaviour.
 *
 *   When user sends "SET,ID:3" via Bluetooth:
 *     attendance.c tells fp_driver to capture 5 images
 *     attendance.c tells fp_storage to save each image
 *     attendance.c sends progress updates via bt_send()
 *
 *   When user sends "VERIFY":
 *     attendance.c captures one new image → saves to tmp file
 *     attendance.c calls fp_compare for each stored user
 *     attendance.c picks best match → sends result via bt_send()
 *     attendance.c logs the event to /fp/attendance.log
 *
 * FLOW DIAGRAMS:
 * ==============
 *
 * SET,ID:n flow:
 *   BT → cmd_parser → attendance_register_user(n)
 *       ↓ (repeat 5 times)
 *       fp_gen_image() → fp_upload_image() → fp_storage_write_callback()
 *       ↓
 *       fp_storage_write_meta()
 *       ↓
 *       bt_send("OK,SET,ID:n,DONE")
 *
 * VERIFY flow:
 *   BT → cmd_parser → attendance_verify()
 *       ↓
 *       fp_gen_image() → fp_upload_image() → /tmp/verify.raw
 *       ↓
 *       for each user: fp_compare_best_for_user()
 *       ↓ (pick best score)
 *       if score >= threshold → bt_send("MATCH,ID:n,SIM:xx%")
 *                               attendance_log_event(id, sim)
 *       else                 → bt_send("NO_MATCH,BEST_SIM:xx%")
 * ============================================================
 */
#ifndef ATTENDANCE_H
#define ATTENDANCE_H

#include <stdint.h>
#include "esp_err.h"

/* ============================================================
 * PUBLIC API — Command Handlers
 * ============================================================
 * Each function handles one Bluetooth command completely:
 * captures, stores, compares, sends response, logs.
 */

/**
 * @brief Handle SET,ID:<n> — Register a new user's fingerprints.
 *
 * Captures FP_IMAGES_PER_USER fingerprint images from sensor,
 * stores each as /fp/fingerprints/<n>/fp<i>.raw,
 * writes metadata to /fp/fingerprints/<n>/meta.txt.
 *
 * Sends progress updates via Bluetooth:
 *   "PLACE FINGER FOR IMAGE 1/5..."
 *   "OK,SET,ID:1,IMG:1,BYTES:36864"
 *   "LIFT FINGER AND PLACE AGAIN"
 *   ... (repeats 5 times)
 *   "OK,SET,ID:1,DONE"
 *
 * @param user_id  User to register (1–255)
 * @return ESP_OK on success
 */
esp_err_t attendance_register_user(uint8_t user_id);

/**
 * @brief Handle DEL,ID:<n> — Delete a registered user.
 *
 * Deletes all fp*.raw files and meta.txt for the user.
 * Sends: "OK,DEL,ID:<n>" or "ERR,USER_NOT_FOUND"
 *
 * @param user_id  User to delete
 */
void attendance_delete_user(uint8_t user_id);

/**
 * @brief Handle VERIFY — Identify a placed finger.
 *
 * Captures one fingerprint image, compares against ALL stored
 * users' images, returns best match result.
 *
 * Sends one of:
 *   "MATCH,ID:<n>,SIM:<xx>%"   — match found, threshold exceeded
 *   "NO_MATCH,BEST_SIM:<xx>%"  — no match above threshold
 *   "ERR,NO_FINGER"            — finger not detected
 *   "ERR,CAPTURE_FAILED"       — image capture error
 *
 * Also logs matched events to /fp/attendance.log.
 */
void attendance_verify(void);

/**
 * @brief Handle LIST — Print all registered user IDs.
 *
 * Sends: "USERS:1,3,5" (comma-separated IDs)
 *        "NO_USERS" if none registered
 */
void attendance_list(void);

/**
 * @brief Handle INFO — Print LittleFS and image statistics.
 *
 * Sends multi-line response:
 *   "FLASH_TOTAL:<n>KB"
 *   "FLASH_USED:<n>KB"
 *   "FLASH_FREE:<n>KB"
 *   "USERS:<n>"
 *   "IMAGES:<n>"
 */
void attendance_info(void);

/**
 * @brief Handle CLEAR — Delete ALL fingerprint data.
 *
 * Sends: "OK,CLEAR,ALL_DELETED" or "ERR,CLEAR_FAILED"
 */
void attendance_clear(void);

/* ============================================================
 * PUBLIC API — Internal Utilities
 * ============================================================ */

/**
 * @brief Log an attendance event to /fp/attendance.log.
 *
 * Appends a line: "USER:<id>,SIM:<sim>%,TIME:<uptime_ms>"
 * File is plain text, viewable over Bluetooth (read file command).
 *
 * @param user_id     Matched user ID
 * @param similarity  Similarity percentage of the match
 */
void attendance_log_event(uint8_t user_id, uint8_t similarity);

/**
 * @brief FreeRTOS task — continuously scans and verifies fingerprints.
 *
 * Runs forever in the background. Does NOT need a BT command.
 * Polls the sensor every FP_AUTO_VERIFY_POLL_MS milliseconds.
 * When a finger is detected, it runs a full verification and logs
 * the result to the ESP-IDF terminal (idf.py monitor).
 *
 * Launch once from app_main() via xTaskCreate().
 *
 * @param arg  Not used (required by FreeRTOS task signature)
 */
void attendance_auto_verify_task(void *arg);

#endif /* ATTENDANCE_H */
