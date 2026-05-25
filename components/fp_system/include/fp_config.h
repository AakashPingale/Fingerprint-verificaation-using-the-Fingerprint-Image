/**
 * @file fp_config.h
 * @brief Centralised compile-time configuration for FP_Module.
 *
 * ============================================================
 * BEGINNER EXPLANATION — HOW TO USE THIS FILE
 * ============================================================
 * Every important number in the firmware lives HERE, not buried
 * in some .c file. To change a setting (e.g., add more images
 * per user or change the UART pins), edit this one file and
 * rebuild. Every other file reads values from here.
 *
 * This "single source of truth" pattern is a best practice in
 * embedded firmware development.
 * ============================================================
 */
#ifndef FP_CONFIG_H
#define FP_CONFIG_H

#include <stdint.h>

/* ============================================================
 * SECTION 1 — UART / SENSOR HARDWARE
 * ============================================================
 * The R307S fingerprint sensor connects to the ESP32 via UART.
 * UART = Universal Asynchronous Receiver Transmitter.
 * It is a simple 2-wire serial link (TX and RX).
 *
 * WIRING DIAGRAM:
 *   R307S                   ESP32
 *   ------                  -----
 *   VCC (3.3V)  --------->  3.3V
 *   GND         --------->  GND
 *   TX          --------->  GPIO16  (ESP32 receives on GPIO16)
 *   RX          --------->  GPIO17  (ESP32 sends on GPIO17)
 *   TOUCH (WAK) --------->  GPIO4   (optional, goes HIGH when finger present)
 */
#define FP_UART_NUM       UART_NUM_2   /* ESP32 has 3 UARTs: 0, 1, 2. Use 2. */
#define FP_TX_PIN         17           /* ESP32 GPIO that sends data TO sensor  */
#define FP_RX_PIN         16           /* ESP32 GPIO that receives FROM sensor  */
#define FP_BAUD_RATE      57600        /* Bits per second — must match R307S    */
#define FP_UART_BUF_SIZE  1024         /* UART hardware RX ring buffer size     */

/* TOUCH pin: R307S WAK output, goes HIGH when a finger is placed on sensor */
#define FP_TOUCH_PIN      4            /* GPIO4 — connect to R307S WAK/TOUCH pin */
#define FP_USE_TOUCH_PIN  1            /* 1 = enable touch interrupt, 0 = disable */

/* ============================================================
 * SECTION 2 — R307S IMAGE SPECIFICATIONS
 * ============================================================
 * The R307S captures fingerprints as a 256×288 pixel image.
 *
 * WHAT IS A GRAYSCALE IMAGE?
 * Each pixel is a brightness value. Black = 0, White = maximum.
 * The sensor uses 4-bit grayscale: 16 brightness levels (0–15).
 *   0  = pure black (deep ridge/valley)
 *   15 = pure white (peak of finger ridge)
 *
 * HOW ARE PIXELS PACKED FOR UART TRANSFER?
 * Sending one pixel as 8 bits wastes 4 bits (only 16 values used).
 * Instead, the sensor packs TWO pixels into ONE byte:
 *   Byte = [Pixel_N : 4 bits HIGH] [Pixel_N+1 : 4 bits LOW]
 *   Byte >> 4       = Pixel N    (bits 7-4)
 *   Byte &  0x0F   = Pixel N+1  (bits 3-0)
 *
 * RESULT: 256×288 pixels / 2 pixels-per-byte = 36,864 bytes
 */
#define FP_IMAGE_WIDTH    256          /* Pixels per horizontal row            */
#define FP_IMAGE_HEIGHT   288          /* Number of rows in the image          */
#define FP_BYTES_PER_ROW  (FP_IMAGE_WIDTH / 2)     /* 128 bytes = one row     */
#define FP_IMAGE_SIZE     (FP_BYTES_PER_ROW * FP_IMAGE_HEIGHT)  /* 36,864 bytes */
#define FP_IMAGE_PIXELS   (FP_IMAGE_WIDTH * FP_IMAGE_HEIGHT)    /* 73,728 pixels */

/* ============================================================
 * SECTION 3 — R307S PACKET PROTOCOL CONSTANTS
 * ============================================================
 * Every communication between ESP32 and R307S is wrapped in a
 * structured packet. Think of a packet as an envelope:
 *   [Header][Address][Type][Length][Data][Checksum]
 *
 * PACKET IDENTIFIERS (PID) — what type of packet is it?
 */
#define FP_HEADER_H       0xEF        /* Byte 0 of every packet header         */
#define FP_HEADER_L       0x01        /* Byte 1 of every packet header         */
#define FP_PID_COMMAND    0x01        /* We send: a command to the sensor      */
#define FP_PID_DATA       0x02        /* Sensor sends: image data chunk        */
#define FP_PID_ACK        0x07        /* Sensor sends: response to our command */
#define FP_PID_END        0x08        /* Sensor sends: last image data packet  */

/* SENSOR RESULT CODES — inside the ACK packet's data field */
#define FP_RC_OK          0x00        /* Command completed successfully        */
#define FP_RC_ERROR       0x01        /* Receive packet error                  */
#define FP_RC_NO_FINGER   0x02        /* Finger not detected on sensor pad     */
#define FP_RC_BAD_IMAGE   0x03        /* Finger detected but image is too poor */

/* ============================================================
 * SECTION 4 — TIMING CONSTANTS
 * ============================================================
 * Timeouts prevent the firmware from hanging if the sensor
 * stops responding (e.g., disconnected or powered off).
 */
#define FP_ACK_TIMEOUT_MS   3000      /* Max wait for sensor to ACK a command  */
#define FP_DATA_TIMEOUT_MS   500      /* Max wait for each image data packet   */
#define FP_IMAGE_TIMEOUT_MS 30000     /* Max wait for full image transfer      */

/* ============================================================
 * SECTION 5 — REGISTRATION PARAMETERS
 * ============================================================
 * WHY 5 IMAGES?
 * No two fingerprint placements are identical. Even the same
 * finger placed twice produces different pixel patterns due to:
 *   - Slight rotation (angle change)
 *   - Different pressure (spreads ridges differently)
 *   - Skin moisture (changes reflection)
 *   - Position offset (translated up/down/left/right)
 *
 * Storing 5 images creates a "coverage set" that captures
 * the natural variation of how a user places their finger.
 * During VERIFY, we compare against ALL 5 and pick the BEST
 * match score, dramatically reducing false rejections.
 */
#define FP_IMAGES_PER_USER   5        /* Number of images to capture per user */
#define FP_MAX_CAPTURE_RETRIES 8      /* Max GenImg attempts before giving up */
#define FP_RETRY_DELAY_MS    800      /* Wait between capture retry attempts   */
#define FP_LIFT_DELAY_MS     2500     /* Pause between consecutive captures    */

/* ============================================================
 * SECTION 6 — FINGERPRINT COMPARISON THRESHOLDS
 * ============================================================
 * WHAT IS SIMILARITY PERCENTAGE?
 * We compare two images pixel by pixel. If every pixel is
 * identical: similarity = 100%. If all pixels are opposite:
 * similarity = 0%. In practice, same-finger images score 70-90%
 * and different-finger images score 30-50%.
 *
 * HISTOGRAM GATE (Stage 1 — fast pre-filter):
 * Before doing the expensive pixel-by-pixel comparison, we first
 * compare the statistical distribution of brightness values
 * (histogram). If histograms are very different, the images
 * are certainly from different fingers — skip Stage 2 entirely.
 * This saves time when checking the wrong finger against many users.
 *
 * Bhattacharyya Coefficient (BC) ranges 0.0 (no overlap) to 1.0
 * (identical histograms). We reject if BC < 0.5.
 *
 * SAD SIMILARITY (Stage 2 — per-pixel comparison):
 * SAD = Sum of Absolute Differences = Σ|pixelA - pixelB|
 * similarity% = (1 - SAD / MAX_POSSIBLE_SAD) × 100
 */
#define FP_DEFAULT_THRESHOLD  70      /* Minimum similarity% to declare MATCH  */
#define FP_HIST_GATE_THRESHOLD 0.50f  /* Bhattacharyya gate: reject if BC < this*/

/* ============================================================
 * SECTION 7 — FILESYSTEM PATHS
 * ============================================================ */
#define FP_FS_MOUNT_POINT   "/fp"            /* VFS mount point for LittleFS  */
#define FP_FS_PARTITION     "littlefs"       /* Partition label in partitions.csv */
#define FP_ROOT_DIR         "/fp/fingerprints"   /* Root of all fingerprint data */
#define FP_TMP_DIR          "/fp/fingerprints/tmp" /* Temp dir for VERIFY captures */
#define FP_ATTEND_LOG       "/fp/attendance.log"   /* Attendance event log        */
#define FP_MAX_PATH         128              /* Maximum file path length (chars)  */

/* ============================================================
 * SECTION 8 — SYSTEM LIMITS
 * ============================================================ */
#define FP_MAX_USERS        16        /* Maximum registered users (filesystem limited) */

/* ============================================================
 * SECTION 9 — BLUETOOTH CONFIGURATION
 * ============================================================
 * The ESP32 acts as a Bluetooth Serial Port Profile (SPP) server.
 * Connect from Android using apps like "Serial Bluetooth Terminal"
 * or write your own app using Android BT SPP API.
 *
 * Device name appears when scanning for Bluetooth devices.
 */
#define BT_DEVICE_NAME      "FP_Attendance"   /* Visible name during BT scan   */
#define BT_RX_QUEUE_SIZE    512               /* Command receive queue (bytes)  */
#define BT_CMD_MAX_LEN      64                /* Maximum command string length  */
#define BT_TX_BUF_SIZE      256               /* Send buffer size (bytes)       */

/* ============================================================
 * SECTION 10 — TASK STACK SIZES
 * ============================================================
 * FreeRTOS tasks each have their own stack. Too small = crash.
 * These values are conservative/safe for beginners.
 */
#define MAIN_TASK_STACK_SIZE   8192   /* Main command processing task          */
#define MAIN_TASK_PRIORITY     5      /* FreeRTOS priority (higher = more CPU) */

#endif /* FP_CONFIG_H */
