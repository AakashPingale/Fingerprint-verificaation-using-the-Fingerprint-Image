/**
 * @file fp_driver.h
 * @brief Layer 3 — R307S Fingerprint Sensor UART Driver Interface.
 *
 * ============================================================
 * BEGINNER EXPLANATION — THE R307S SENSOR
 * ============================================================
 * The R307S is a capacitive optical fingerprint sensor module.
 * It has its own processor that handles image capture.
 *
 * INTERNAL SENSOR MEMORY:
 *   ImageBuffer — where the captured fingerprint image is stored
 *                 (inside the sensor, not in ESP32 RAM)
 *
 * COMMANDS WE USE (and why):
 *   GenImg (0x01):  "Sensor, take a fingerprint photo NOW"
 *     ↳ The sensor activates the optical scanner, captures the
 *       image, and stores it in its ImageBuffer.
 *     ↳ Returns: OK / No-Finger / Bad-Image
 *
 *   UpImage (0x0A): "Sensor, SEND me the photo over UART"
 *     ↳ The sensor reads its ImageBuffer and streams the
 *       36,864 bytes of 4-bit pixel data to the ESP32.
 *     ↳ Data arrives as ~289 packets (288 data + 1 end).
 *
 * COMMANDS WE DO NOT USE (and why):
 *   Img2Tz — converts image to proprietary feature template
 *   RegModel — merges two templates into one biometric model
 *   Match / Search — compares templates using sensor's engine
 *   Store — saves template to sensor's internal database
 *
 *   We bypass all of these because we want full control over
 *   image storage and comparison logic (image-based biometrics).
 *
 * STREAMING CALLBACK:
 *   fp_upload_image() does not buffer the 36KB image in RAM.
 *   Instead, it calls your callback for each ~128-byte chunk.
 *   Your callback writes that chunk directly to LittleFS flash.
 *   This is called "streaming" — processes data as it arrives.
 * ============================================================
 */
#ifndef FP_DRIVER_H
#define FP_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

/* ============================================================
 * DRIVER ERROR CODES
 * ============================================================
 * Using named constants instead of magic numbers makes errors
 * self-documenting. Compare: return 3 vs return FP_ERR_TIMEOUT
 */
#define FP_OK               0    /* Operation completed successfully           */
#define FP_ERR_NO_FINGER    1    /* No finger is placed on the sensor pad      */
#define FP_ERR_BAD_IMAGE    2    /* Image captured but quality is too poor     */
#define FP_ERR_TIMEOUT      3    /* Sensor did not respond within timeout      */
#define FP_ERR_CHECKSUM     4    /* Packet checksum mismatch (data corruption) */
#define FP_ERR_UART         5    /* UART hardware/write failure                */
#define FP_ERR_SENSOR       6    /* Sensor returned an unexpected error code   */
#define FP_ERR_INVALID      7    /* Function called with invalid parameter     */
#define FP_ERR_STORAGE      8    /* Filesystem read/write error                */

/* ============================================================
 * STREAMING DATA CALLBACK TYPE
 * ============================================================
 * DESIGN PATTERN EXPLANATION:
 * Instead of allocating a 36KB buffer for the full image (which
 * would use ~11% of ESP32's total RAM), we use a callback:
 *
 *   fp_upload_image(my_write_function, my_file_handle, &bytes);
 *
 * For each 128-byte chunk received from UART, the driver calls:
 *   my_write_function(data, 128, my_file_handle)
 *
 * YOUR callback writes those 128 bytes to flash, then returns.
 * The driver goes on to receive the next 128 bytes.
 * Total RAM used for image: just 128 bytes at a time! ✅
 *
 * @param data    Pointer to received pixel bytes (4-bit packed, 2px/byte)
 * @param len     Number of bytes in this chunk (up to 128)
 * @param ctx     User context (e.g., your open FILE* handle)
 * @return        0 = success; non-zero = abort transfer with error
 */
typedef int (*fp_data_callback_t)(const uint8_t *data, size_t len, void *ctx);

/* ============================================================
 * PUBLIC API
 * ============================================================ */

/**
 * @brief Initialize UART2 for R307S communication.
 *
 * Must be called once at startup, before any fp_ functions.
 * Configures: baud rate 57600, 8N1, GPIO pins from fp_config.h
 * Optionally configures TOUCH pin interrupt if FP_USE_TOUCH_PIN=1
 *
 * @return ESP_OK on success
 */
esp_err_t fp_driver_init(void);

/**
 * @brief Deinitialize UART driver (call at shutdown if needed).
 */
void fp_driver_deinit(void);

/**
 * @brief Send GenImg command — capture one fingerprint image.
 *
 * After this succeeds, the image is held in the sensor's
 * internal ImageBuffer, ready for fp_upload_image().
 *
 * RETRY RECOMMENDED: If a finger is being lowered, the first
 * call might return FP_ERR_NO_FINGER. Call again after 300ms.
 *
 * @return FP_OK            — image captured, ready to upload
 *         FP_ERR_NO_FINGER — no finger detected
 *         FP_ERR_BAD_IMAGE — image too blurry/messy
 *         FP_ERR_TIMEOUT   — sensor not responding
 */
int fp_gen_image(void);

/**
 * @brief Send UpImage command — stream sensor ImageBuffer to ESP32.
 *
 * MUST call fp_gen_image() successfully before calling this.
 * Streams 36,864 bytes (288 packets × 128 bytes) via callback.
 * Each callback call must return 0 (success) to continue.
 *
 * @param cb         Write callback called for each image chunk
 * @param cb_ctx     Arbitrary context pointer passed to cb
 * @param bytes_out  [Optional] Receives total bytes transferred
 * @return FP_OK on success, FP_ERR_* on failure
 */
int fp_upload_image(fp_data_callback_t cb, void *cb_ctx, size_t *bytes_out);

/**
 * @brief Flush UART receive buffer — discard all pending bytes.
 * Call this to recover from corrupted communication states.
 */
void fp_uart_flush(void);

/**
 * @brief Attempt to control the sensor's LED (Aura control - 0x35 command).
 * 
 * @param blinking true for continuous blinking, false to turn off
 * @return FP_OK if accepted, FP_ERR_SENSOR (0x0B) if unsupported by firmware
 */
int fp_set_led_mode(bool blinking);

/**
 * @brief Read exactly one byte from UART with timeout.
 * @return true if byte received, false on timeout
 */
bool fp_uart_read_byte(uint8_t *byte_out, uint32_t timeout_ms);

/**
 * @brief Perform a live handshake (Verify Password 0x13) to check if the module is connected.
 * 
 * @return FP_OK on success, FP_ERR_TIMEOUT if no response, FP_ERR_SENSOR on wrong response/password.
 */
int fp_detect_module(void);

/**
 * @brief Get the cached fingerprint module connection state.
 * 
 * @return true if detected, false if not detected.
 */
bool fp_is_detected(void);

#endif /* FP_DRIVER_H */

