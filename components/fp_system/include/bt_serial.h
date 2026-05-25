/**
 * @file bt_serial.h
 * @brief Layer 1 — Bluetooth Classic SPP Serial Interface.
 *
 * ============================================================
 * BEGINNER EXPLANATION — WHAT THIS LAYER DOES
 * ============================================================
 * This is the LOWEST software layer visible to the user. It
 * handles the raw Bluetooth connection:
 *
 *   Phone/PC  <──── Bluetooth SPP ────>  ESP32
 *                                          │
 *                                     bt_serial.c
 *                                          │
 *                                    (bytes arrive here)
 *                                          │
 *                                     cmd_parser.c
 *                                     (interprets commands)
 *
 * SPP = Serial Port Profile. It creates a virtual serial cable
 * over Bluetooth. You pair with "FP_Attendance" in your phone's
 * Bluetooth settings, then use any terminal app to send commands.
 *
 * THREAD SAFETY:
 * The BT stack delivers data in its own internal task. To avoid
 * race conditions, received bytes go into a FreeRTOS queue.
 * bt_wait_command() reads from that queue safely from any task.
 * ============================================================
 */
#ifndef BT_SERIAL_H
#define BT_SERIAL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

/* ============================================================
 * PUBLIC API
 * ============================================================ */

/**
 * @brief Initialize Bluetooth Classic SPP server.
 *
 * This function:
 *  1. Releases BLE memory (we only use Classic BT, not BLE)
 *  2. Initialises the Bluedroid BT stack
 *  3. Registers the GAP callback (handles pairing)
 *  4. Registers the SPP callback (handles connection/data)
 *  5. Starts an SPP server named BT_DEVICE_NAME
 *
 * After calling this, the ESP32 is discoverable. Pair with it
 * from your phone and connect using a BT terminal app.
 *
 * @return ESP_OK on success, ESP_FAIL on any BT init error
 */
esp_err_t bt_serial_init(void);

/**
 * @brief Wait for a complete command line from Bluetooth.
 *
 * Blocks until a newline-terminated string arrives or timeout.
 * Leading/trailing whitespace is stripped. Empty lines ignored.
 *
 * EXAMPLE: User sends "SET,ID:1\r\n"
 *          buf = "SET,ID:1", returns true
 *
 * @param buf         Buffer to store received command
 * @param buf_size    Size of the buffer in bytes
 * @param timeout_ms  How long to wait (portMAX_DELAY = wait forever)
 * @return true if a command was received, false on timeout
 */
bool bt_wait_command(char *buf, int buf_size, uint32_t timeout_ms);

/**
 * @brief Send a null-terminated string over Bluetooth.
 *
 * Thread-safe. Can be called from any task.
 * If no client is connected, the call is silently ignored.
 *
 * @param str  Null-terminated string to send
 */
void bt_send(const char *str);

/**
 * @brief Send a formatted string over Bluetooth (printf-style).
 *
 * EXAMPLE: bt_sendf("MATCH,ID:%d,SIM:%d%%\r\n", uid, sim);
 *
 * @param fmt  printf-format string
 * @param ...  Format arguments
 */
void bt_sendf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/**
 * @brief Returns true if a Bluetooth client is currently connected.
 */
bool bt_is_connected(void);

#endif /* BT_SERIAL_H */
