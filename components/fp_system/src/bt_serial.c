/**
 * @file bt_serial.c
 * @brief Layer 1 — Bluetooth Classic SPP Server Implementation.
 *
 * ============================================================
 * HOW BLUETOOTH CLASSIC SPP WORKS ON ESP32
 * ============================================================
 * 1. We initialize the Bluedroid Bluetooth stack (Espressif's
 *    open-source BT implementation, based on Android Bluedroid).
 *
 * 2. We register as an SPP (Serial Port Profile) server.
 *    SPP emulates an old RS-232 serial cable over Bluetooth.
 *    Your phone sees it as a "Bluetooth serial device".
 *
 * 3. When a phone connects:
 *    - ESP_SPP_SRV_OPEN_EVT fires → we save the connection handle
 *    - We can now call esp_spp_write() to send data to phone
 *
 * 4. When phone sends data:
 *    - ESP_SPP_DATA_IND_EVT fires with the received bytes
 *    - We push bytes into a FreeRTOS queue (thread-safe)
 *    - bt_wait_command() reads from this queue in the main task
 *
 * THREAD SAFETY:
 * The BT stack runs in its own FreeRTOS task context.
 * Our application runs in the main task context.
 * The FreeRTOS queue bridges them safely.
 * ============================================================
 */
#include "bt_serial.h"
#include "fp_config.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

static const char *TAG = "bt_serial";

/* ============================================================
 * MODULE-LEVEL STATE (static = private to this file)
 * ============================================================ */

/* FreeRTOS byte queue for received characters.
 * The BT callback pushes bytes here. bt_wait_command() pops them. */
static QueueHandle_t s_rx_queue = NULL;

/* SPP connection handle. Set when client connects, cleared on disconnect.
 * 0 means "no client connected". Used by bt_send() to know who to write to. */
static uint32_t s_conn_handle = 0;

/* ============================================================
 * GAP CALLBACK — Handles Bluetooth Pairing/Authentication
 * ============================================================
 * GAP = Generic Access Profile. Handles discoverability and pairing.
 * BEGINNER NOTE: When a phone pairs with our device for the first time,
 * this callback handles the authentication (PIN code if needed).
 */
static void gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {

    case ESP_BT_GAP_AUTH_CMPL_EVT:
        /* Pairing completed */
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Paired successfully with: %s",
                     param->auth_cmpl.device_name);
        } else {
            ESP_LOGW(TAG, "Pairing failed, status: %d",
                     param->auth_cmpl.stat);
        }
        break;

    case ESP_BT_GAP_PIN_REQ_EVT: {
        /* Phone is asking for a PIN code.
         * We respond with "1234". For production, use a secure PIN. */
        ESP_LOGI(TAG, "PIN requested from remote device");
        esp_bt_pin_code_t pin = {'1','2','3','4', 0};
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin);
        break;
    }

    case ESP_BT_GAP_CFM_REQ_EVT:
        /* Numeric comparison confirmation (Bluetooth 2.1+).
         * Auto-confirm for convenience. */
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;

    default:
        ESP_LOGD(TAG, "GAP event: %d", event);
        break;
    }
}

/* ============================================================
 * SPP CALLBACK — Handles SPP Connection Events and Data
 * ============================================================
 * SPP = Serial Port Profile.
 * This callback fires for every significant SPP event:
 *   - Initialization complete
 *   - Client connected
 *   - Data received
 *   - Client disconnected
 */
static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {

    case ESP_SPP_INIT_EVT:
        /* SPP stack initialized — now safe to start the server.
         * We set the device visible/connectable and start listening. */
        ESP_LOGI(TAG, "SPP initialized, starting server: %s", BT_DEVICE_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        esp_spp_start_srv(ESP_SPP_SEC_NONE,     /* No security/PIN for simplicity */
                          ESP_SPP_ROLE_SLAVE,    /* We are the server (slave) */
                          0,                     /* Channel: 0 = auto-assign */
                          BT_DEVICE_NAME);       /* Service name shown to clients */
        break;

    case ESP_SPP_SRV_OPEN_EVT:
        /* A client (phone/PC) connected to our SPP server */
        s_conn_handle = param->srv_open.handle;
        ESP_LOGI(TAG, "Client connected, handle=%lu", (unsigned long)s_conn_handle);
        /* Greet the client */
        bt_send("\r\n=== FP_Attendance System Ready ===\r\n");
        bt_send("Commands: SET,ID:<n> | DEL,ID:<n> | VERIFY | LIST | INFO | CLEAR\r\n");
        bt_send("> ");
        break;

    case ESP_SPP_CLOSE_EVT:
        /* Client disconnected — reset handle */
        ESP_LOGI(TAG, "Client disconnected, handle=%lu",
                 (unsigned long)param->close.handle);
        s_conn_handle = 0;
        /* Flush the queue in case of partially received command */
        if (s_rx_queue) {
            xQueueReset(s_rx_queue);
        }
        break;

    case ESP_SPP_DATA_IND_EVT: {
        /* Data received from client.
         * Push each byte into the RX queue.
         * bt_wait_command() will reassemble them into complete lines. */
        uint8_t *data = param->data_ind.data;
        uint16_t len  = param->data_ind.len;

        ESP_LOGD(TAG, "RX %d bytes", len);

        for (uint16_t i = 0; i < len; i++) {
            /* Non-blocking push — if queue is full, drop the byte */
            if (xQueueSend(s_rx_queue, &data[i], 0) != pdTRUE) {
                ESP_LOGW(TAG, "RX queue full, dropping byte 0x%02X", data[i]);
            }
        }
        break;
    }

    case ESP_SPP_WRITE_EVT:
        /* Write completed callback — not needed for our design */
        ESP_LOGD(TAG, "Write complete, cong=%d", param->write.cong);
        break;

    case ESP_SPP_CONG_EVT:
        /* Congestion notification — BT buffer is full/cleared */
        ESP_LOGD(TAG, "BT congestion: %s", param->cong.cong ? "CONGESTED" : "CLEAR");
        break;

    default:
        ESP_LOGD(TAG, "SPP event: %d", event);
        break;
    }
}

/* ============================================================
 * bt_serial_init — Initialize Bluetooth Classic SPP
 * ============================================================ */
esp_err_t bt_serial_init(void)
{
    esp_err_t err;

    /* Create the receive queue FIRST (before BT callbacks start firing) */
    s_rx_queue = xQueueCreate(BT_RX_QUEUE_SIZE, sizeof(uint8_t));
    if (!s_rx_queue) {
        ESP_LOGE(TAG, "Failed to create RX queue");
        return ESP_ERR_NO_MEM;
    }

    /* STEP 1: Skip BLE memory release to prevent ESP_ERR_INVALID_ARG mode conflicts in ESP-IDF v5.3 */
    // err = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

    /* STEP 2: Initialize the BT controller hardware */
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&bt_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(err));
        return err;
    }

    /* STEP 3: Enable the BT controller for Classic BT mode */
    err = esp_bt_controller_enable(bt_cfg.mode);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(err));
        return err;
    }

    /* STEP 4: Initialize Bluedroid (the BT host stack) */
    err = esp_bluedroid_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(err));
        return err;
    }

    /* STEP 5: Enable Bluedroid */
    err = esp_bluedroid_enable();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(err));
        return err;
    }

    /* STEP 6: Set the device name (visible when scanning from phone) */
    err = esp_bt_gap_set_device_name(BT_DEVICE_NAME);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Set device name failed: %s", esp_err_to_name(err));
        return err;
    }

    /* STEP 7: Register GAP callback (handles pairing) */
    err = esp_bt_gap_register_callback(gap_callback);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GAP callback register failed: %s", esp_err_to_name(err));
        return err;
    }

    /* STEP 8: Set secure simple pairing mode */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;  /* Can display and input */
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(iocap));

    /* STEP 9: Register SPP callback (handles connection/data events) */
    err = esp_spp_register_callback(spp_callback);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPP callback register failed: %s", esp_err_to_name(err));
        return err;
    }

    /* STEP 10: Initialize SPP in callback mode.
     * ESP_SPP_MODE_CB = events delivered via callback (not polling). */
    esp_spp_cfg_t spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size = 0  /* Only used for VFS mode */
    };
    err = esp_spp_enhanced_init(&spp_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPP init failed: %s", esp_err_to_name(err));
        return err;
    }

    /* The SPP server is started inside spp_callback when ESP_SPP_INIT_EVT fires */
    ESP_LOGI(TAG, "BT init OK — device name: %s", BT_DEVICE_NAME);
    ESP_LOGI(TAG, "Waiting for Bluetooth connection...");

    return ESP_OK;
}

/* ============================================================
 * bt_wait_command — Block until a complete command line arrives
 * ============================================================
 * Reads bytes from the queue one at a time, assembling them
 * into a complete command string (terminated by \n or \r).
 * Empty lines and pure whitespace are ignored.
 */
bool bt_wait_command(char *buf, int buf_size, uint32_t timeout_ms)
{
    int pos = 0;
    uint8_t byte;

    /* Convert timeout to FreeRTOS ticks.
     * portMAX_DELAY = wait forever (until data arrives). */
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    while (true) {
        /* Calculate remaining ticks */
        TickType_t now = xTaskGetTickCount();
        TickType_t remaining = (deadline > now) ? (deadline - now) : 0;

        if (remaining == 0 && timeout_ms != portMAX_DELAY) {
            buf[pos] = '\0';
            return (pos > 0);  /* Timeout: return whatever we have */
        }

        /* Block waiting for the next byte from BT */
        TickType_t wait = (timeout_ms == portMAX_DELAY) ? portMAX_DELAY : remaining;
        if (xQueueReceive(s_rx_queue, &byte, wait) != pdTRUE) {
            buf[pos] = '\0';
            return false;  /* Timeout with no data */
        }

        char c = (char)byte;

        if (c == '\n' || c == '\r') {
            /* End of line — command complete if non-empty */
            if (pos > 0) {
                buf[pos] = '\0';
                return true;
            }
            /* Empty line — ignore and keep waiting */
            continue;
        }

        if (c >= 0x20 && c < 0x7F) {
            /* Printable ASCII character — add to buffer */
            if (pos < buf_size - 1) {
                buf[pos++] = c;
            }
        }
        /* Ignore control characters other than \n \r */
    }
}

/* ============================================================
 * bt_send — Send a string to the connected BT client
 * ============================================================ */
void bt_send(const char *str)
{
    if (s_conn_handle == 0 || str == NULL) return;

    int len = strlen(str);
    if (len == 0) return;

    /* esp_spp_write() copies data into BT stack's internal buffer.
     * It is safe to call from any task context. */
    esp_err_t err = esp_spp_write(s_conn_handle, len, (uint8_t *)str);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "bt_send failed: %s", esp_err_to_name(err));
    }
}

/* ============================================================
 * bt_sendf — Printf-style formatted BT send
 * ============================================================ */
void bt_sendf(const char *fmt, ...)
{
    char buf[BT_TX_BUF_SIZE];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    bt_send(buf);
}

/* ============================================================
 * bt_is_connected — Check if client is currently connected
 * ============================================================ */
bool bt_is_connected(void)
{
    return (s_conn_handle != 0);
}
