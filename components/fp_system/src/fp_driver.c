/**
 * @file fp_driver.c
 * @brief Layer 3 — R307S Fingerprint Sensor UART Driver Implementation.
 *
 * ============================================================
 * DETAILED EXPLANATION: UART COMMUNICATION WITH R307S
 * ============================================================
 *
 * UART BASICS FOR BEGINNERS:
 * --------------------------
 * UART is the simplest serial communication protocol.
 * Data is sent as a stream of bits at a fixed speed (baud rate).
 * At 57600 baud:
 *   - Each bit lasts: 1/57600 = 17.36 microseconds
 *   - Each byte (8 bits + start/stop) lasts: 10 × 17.36 = 173.6 µs
 *   - One 128-byte packet arrives in: 128 × 173.6 µs = 22.2 ms
 *   - Full 36KB image takes: 36864 × 173.6 µs = 6.4 seconds
 *
 * PACKET STRUCTURE DETAIL:
 * ------------------------
 * Every communication is wrapped in a fixed-format packet:
 *
 *  Byte  0: 0xEF (Header byte 1)
 *  Byte  1: 0x01 (Header byte 2)
 *  Byte  2: 0xFF (Address byte 0, default)
 *  Byte  3: 0xFF (Address byte 1, default)
 *  Byte  4: 0xFF (Address byte 2, default)
 *  Byte  5: 0xFF (Address byte 3, default)
 *  Byte  6: PID  (Packet type: 0x01=cmd, 0x02=data, 0x07=ack, 0x08=end)
 *  Byte  7: LEN_H (High byte of length field)
 *  Byte  8: LEN_L (Low byte of length field)
 *  Byte  9…N: DATA (command/parameter/image bytes)
 *  Byte N+1: CKSUM_H (checksum high byte)
 *  Byte N+2: CKSUM_L (checksum low byte)
 *
 * LENGTH FIELD value = (number of DATA bytes) + 2 (for the 2 checksum bytes)
 *
 * CHECKSUM CALCULATION:
 * ---------------------
 * Checksum = PID + LEN_H + LEN_L + DATA[0] + DATA[1] + ... + DATA[n]
 * It's a simple 16-bit sum (no CRC, no polynomial — just addition).
 * Transmitted as big-endian (high byte first).
 *
 * WHY CHECKSUM MATTERS:
 * UART has no hardware error correction. If electrical noise corrupts
 * one bit in the 36KB image transfer, we'd get wrong pixels stored.
 * The checksum catches most (not all) corruption cases.
 * If checksum fails, we return FP_ERR_CHECKSUM and the caller
 * can abort/retry.
 *
 * GENIMG COMMAND EXAMPLE:
 * -----------------------
 * Send: EF 01 FF FF FF FF 01 00 03 01 00 05
 *       ^^^^^ ^^^^^^^^^^^ ^^ ^^^^^ ^^ ^^^^^
 *       Header  Address   PID  Len  Cmd Checksum
 *
 * Checksum = 0x01 + 0x00 + 0x03 + 0x01 = 0x05 ✓
 *
 * UPIMAGE COMMAND EXAMPLE:
 * ------------------------
 * Send: EF 01 FF FF FF FF 01 00 03 0A 00 0E
 * Checksum = 0x01 + 0x00 + 0x03 + 0x0A = 0x0E ✓
 *
 * STATE MACHINE EXPLANATION:
 * --------------------------
 * We parse packets byte-by-byte using a finite state machine.
 * A state machine has:
 *   - A set of STATES (where we are in parsing)
 *   - TRANSITIONS (how we move between states when a byte arrives)
 *
 * This is robust: if bytes are garbled, we keep looking for
 * the next valid 0xEF header without getting stuck.
 * ============================================================
 */
#include "fp_driver.h"
#include "fp_config.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "fp_driver";

/* Cached hardware connection status of the fingerprint module */
static bool s_module_detected = false;


/* ============================================================
 * INTERNAL STRUCTURES
 * ============================================================ */

/** Maximum data payload per packet:
 *  Image data packets carry up to 128 bytes.
 *  ACK packets carry 1 byte (result code).
 *  We use 256 for safety margin. */
#define FP_MAX_PACKET_DATA 256

/** Internal packet structure — holds one fully-parsed packet */
typedef struct {
    uint8_t  pid;                        /* Packet type ID (0x02, 0x07, etc.)  */
    uint16_t length;                     /* Raw length field value from packet  */
    uint16_t data_len;                   /* Computed: length - 2 (payload bytes)*/
    uint8_t  data[FP_MAX_PACKET_DATA];   /* Payload bytes                       */
} fp_packet_t;

/** Packet receive state machine states */
typedef enum {
    STATE_HEADER1 = 0,  /* Waiting for first header byte (0xEF)                */
    STATE_HEADER2,      /* Waiting for second header byte (0x01)               */
    STATE_ADDR0,        /* Consuming 4 address bytes (we ignore their values)  */
    STATE_ADDR1,
    STATE_ADDR2,
    STATE_ADDR3,
    STATE_PID,          /* Next byte is the packet type identifier             */
    STATE_LEN_H,        /* Next byte is high byte of length field              */
    STATE_LEN_L,        /* Next byte is low byte of length field               */
    STATE_DATA,         /* Collecting payload bytes (count from length)        */
    STATE_CKSUM_H,      /* Next byte is checksum high byte                     */
    STATE_CKSUM_L,      /* Next byte is checksum low byte → verify and done    */
} pkt_state_t;

/* ============================================================
 * PRIVATE HELPER: Build and send a command packet over UART
 * ============================================================
 *
 * PACKET CONSTRUCTION:
 *  [Header 2B][Addr 4B][PID 1B][Len 2B][Instr 1B][Params NB][Cksum 2B]
 *
 * @param instruction  Command code (0x01=GenImg, 0x0A=UpImage, etc.)
 * @param params       Parameter bytes array (can be NULL if params_len=0)
 * @param params_len   Number of parameter bytes
 */
static void send_command(uint8_t instruction,
                         const uint8_t *params, uint8_t params_len)
{
    /* Maximum command packet: 2+4+1+2+1+4params+2 = 16 bytes */
    uint8_t buf[32];
    int     pos = 0;

    /* Fixed header — every R307S packet starts with these 6 bytes */
    buf[pos++] = FP_HEADER_H;  /* 0xEF */
    buf[pos++] = FP_HEADER_L;  /* 0x01 */
    buf[pos++] = 0xFF;          /* Device address byte 0 (default) */
    buf[pos++] = 0xFF;          /* Device address byte 1 (default) */
    buf[pos++] = 0xFF;          /* Device address byte 2 (default) */
    buf[pos++] = 0xFF;          /* Device address byte 3 (default) */

    /* PID = 0x01 for all commands we send to the sensor */
    buf[pos++] = FP_PID_COMMAND;

    /* Length = instruction(1) + params(N) + checksum(2) */
    uint16_t length = (uint16_t)(1 + params_len + 2);
    buf[pos++] = (uint8_t)(length >> 8);    /* Length HIGH byte */
    buf[pos++] = (uint8_t)(length & 0xFF);  /* Length LOW  byte */

    /* Instruction code */
    buf[pos++] = instruction;

    /* Optional parameter bytes */
    for (int i = 0; i < params_len; i++) {
        buf[pos++] = params[i];
    }

    /* Checksum = sum of: PID + LEN_H + LEN_L + INSTRUCTION + all PARAMS
     * IMPORTANT: checksum does NOT include header bytes or address bytes */
    uint16_t cksum = (uint16_t)FP_PID_COMMAND
                   + (uint16_t)(length >> 8)
                   + (uint16_t)(length & 0xFF)
                   + (uint16_t)instruction;
    for (int i = 0; i < params_len; i++) {
        cksum += (uint16_t)params[i];
    }
    buf[pos++] = (uint8_t)(cksum >> 8);    /* Checksum HIGH byte */
    buf[pos++] = (uint8_t)(cksum & 0xFF);  /* Checksum LOW  byte */

    /* Transmit via UART — blocking write */
    int written = uart_write_bytes(FP_UART_NUM, (const char *)buf, pos);
    ESP_LOGD(TAG, "TX command 0x%02X: %d bytes, cksum=0x%04X",
             instruction, written, cksum);
}

/* ============================================================
 * PRIVATE HELPER: Receive one complete packet using state machine
 * ============================================================
 *
 * HOW THE STATE MACHINE WORKS:
 *
 * We call uart_read_bytes() repeatedly, reading ONE byte at a time.
 * Based on the current 'state', we decide what to do with the byte:
 *
 *   STATE_HEADER1 → received 0xEF? → go to STATE_HEADER2
 *                   got anything else? → stay in STATE_HEADER1
 *
 *   STATE_HEADER2 → received 0x01? → go to STATE_ADDR0
 *                   got 0xEF? → stay in STATE_HEADER2 (could be next header)
 *                   got anything else? → back to STATE_HEADER1
 *
 *   STATE_ADDR0..3 → consume 4 address bytes (we ignore their values)
 *
 *   STATE_PID → save byte as packet type, start checksum accumulation
 *
 *   STATE_LEN_H/L → save 2-byte length, compute data_len = length - 2
 *
 *   STATE_DATA → collect data_len bytes into pkt->data[]
 *
 *   STATE_CKSUM_H/L → receive 2-byte checksum, compare with computed sum
 *                     → return FP_OK or FP_ERR_CHECKSUM
 *
 * @param pkt         Output: filled packet structure on FP_OK
 * @param timeout_ms  Total time budget in milliseconds
 * @return FP_OK, FP_ERR_TIMEOUT, or FP_ERR_CHECKSUM
 */
static int receive_packet(fp_packet_t *pkt, uint32_t timeout_ms)
{
    pkt_state_t state          = STATE_HEADER1;
    int         data_idx       = 0;
    uint16_t    checksum_calc  = 0;   /* Running checksum as bytes arrive    */
    uint16_t    checksum_recv_h = 0;  /* High byte of received checksum      */
    uint8_t     b;                    /* Current byte being processed        */

    /* Deadline as absolute microsecond timestamp */
    int64_t deadline_us = esp_timer_get_time() + (int64_t)timeout_ms * 1000LL;

    while (esp_timer_get_time() < deadline_us) {

        /* Remaining time in milliseconds (capped at 50ms per read call) */
        int64_t remaining_us = deadline_us - esp_timer_get_time();
        if (remaining_us <= 0) break;
        uint32_t read_ms = (uint32_t)( remaining_us / 1000LL );
        if (read_ms > 50) read_ms = 50;   /* Don't block longer than 50ms */

        /* Read one byte — returns 1 on success, 0 on timeout */
        int got = uart_read_bytes(FP_UART_NUM, &b, 1,
                                  pdMS_TO_TICKS(read_ms + 1));
        if (got <= 0) {
            continue;   /* No byte this time — loop and try again */
        }

        /* ── Process the byte based on current state ── */
        switch (state) {

        /* ── HEADER SYNCHRONISATION ── */
        case STATE_HEADER1:
            if (b == FP_HEADER_H) {          /* Found 0xEF */
                state = STATE_HEADER2;
            }
            /* Any other byte: ignore and keep searching */
            break;

        case STATE_HEADER2:
            if (b == FP_HEADER_L) {          /* Found 0x01 — valid header */
                state = STATE_ADDR0;
            } else if (b == FP_HEADER_H) {   /* Could be: EF EF 01 ... */
                state = STATE_HEADER2;        /* Stay on HEADER2         */
            } else {
                state = STATE_HEADER1;        /* Not valid, restart       */
            }
            break;

        /* ── ADDRESS BYTES (ignore, just consume 4 bytes) ── */
        case STATE_ADDR0: state = STATE_ADDR1; break;
        case STATE_ADDR1: state = STATE_ADDR2; break;
        case STATE_ADDR2: state = STATE_ADDR3; break;
        case STATE_ADDR3:
            state = STATE_PID;
            /* Reset state for new packet */
            checksum_calc = 0;
            data_idx = 0;
            break;

        /* ── PACKET TYPE IDENTIFIER ── */
        case STATE_PID:
            pkt->pid = b;
            checksum_calc = (uint16_t)b;   /* Checksum starts accumulating from PID */
            state = STATE_LEN_H;
            break;

        /* ── LENGTH FIELD (2 bytes, big-endian) ── */
        case STATE_LEN_H:
            pkt->length = (uint16_t)b << 8;
            checksum_calc += (uint16_t)b;
            state = STATE_LEN_L;
            break;

        case STATE_LEN_L:
            pkt->length |= (uint16_t)b;
            checksum_calc += (uint16_t)b;

            /* Validate: length must be at least 2 (for the checksum bytes themselves) */
            if (pkt->length < 2) {
                ESP_LOGW(TAG, "Packet length %d too small, resyncing", pkt->length);
                state = STATE_HEADER1;
                break;
            }

            pkt->data_len = pkt->length - 2;  /* Payload = length minus checksum */

            if (pkt->data_len > FP_MAX_PACKET_DATA) {
                ESP_LOGW(TAG, "Packet payload %d exceeds buffer, resyncing",
                         pkt->data_len);
                state = STATE_HEADER1;
                break;
            }

            data_idx = 0;
            /* Skip STATE_DATA if no payload (checksum immediately follows) */
            state = (pkt->data_len > 0) ? STATE_DATA : STATE_CKSUM_H;
            break;

        /* ── PAYLOAD DATA BYTES ── */
        case STATE_DATA:
            pkt->data[data_idx++] = b;
            checksum_calc += (uint16_t)b;
            if (data_idx >= (int)pkt->data_len) {
                state = STATE_CKSUM_H;
            }
            break;

        /* ── CHECKSUM VERIFICATION ── */
        case STATE_CKSUM_H:
            checksum_recv_h = (uint16_t)b << 8;
            state = STATE_CKSUM_L;
            break;

        case STATE_CKSUM_L: {
            uint16_t checksum_recv = checksum_recv_h | (uint16_t)b;

            if (checksum_recv == checksum_calc) {
                /* CHECKSUM MATCH — packet is valid */
                ESP_LOGD(TAG, "Packet OK: PID=0x%02X data_len=%d cksum=0x%04X",
                         pkt->pid, pkt->data_len, checksum_calc);
                return FP_OK;
            } else {
                /* CHECKSUM MISMATCH — data was corrupted in transit */
                ESP_LOGW(TAG, "Checksum FAIL: expected=0x%04X got=0x%04X (PID=0x%02X)",
                         checksum_calc, checksum_recv, pkt->pid);
                return FP_ERR_CHECKSUM;
            }
        }

        default:
            state = STATE_HEADER1;
            break;
        } /* end switch */
    } /* end while */

    ESP_LOGW(TAG, "receive_packet: TIMEOUT after %lu ms (state=%d)",
             (unsigned long)timeout_ms, state);
    return FP_ERR_TIMEOUT;
}

/* ============================================================
 * fp_driver_init — Initialize UART2 for R307S communication
 * ============================================================ */
esp_err_t fp_driver_init(void)
{
    ESP_LOGI(TAG, "FP driver init: UART%d, TX=GPIO%d, RX=GPIO%d, %d baud",
             FP_UART_NUM, FP_TX_PIN, FP_RX_PIN, FP_BAUD_RATE);

    esp_err_t err;

    /* UART configuration structure.
     * 8N1 = 8 data bits, No parity, 1 stop bit — the universal standard. */
    uart_config_t cfg = {
        .baud_rate  = FP_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* Install UART driver.
     * Parameters: UART port, RX buf size, TX buf size, queue size, queue, flags
     * We use 0 for TX buffer (blocking TX) and no event queue (we poll). */
    err = uart_driver_install(FP_UART_NUM,
                              FP_UART_BUF_SIZE,  /* RX ring buffer              */
                              0,                  /* TX buffer (0 = blocking TX) */
                              0, NULL, 0);         /* No event queue              */
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Apply configuration (baud rate, data format) */
    err = uart_param_config(FP_UART_NUM, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Assign GPIO pins to UART peripheral.
     * UART_PIN_NO_CHANGE = don't configure RTS/CTS (we don't use hardware flow control) */
    err = uart_set_pin(FP_UART_NUM,
                       FP_TX_PIN,           /* Our TX → sensor RX */
                       FP_RX_PIN,           /* Our RX ← sensor TX */
                       UART_PIN_NO_CHANGE,   /* RTS: not used      */
                       UART_PIN_NO_CHANGE);  /* CTS: not used      */
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Optional: Configure TOUCH/WAK pin as interrupt input.
     * The R307S WAK pin goes HIGH when a finger is detected.
     * With FP_USE_TOUCH_PIN=1 in fp_config.h, this enables the GPIO. */
#if FP_USE_TOUCH_PIN
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << FP_TOUCH_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,  /* Default to LOW (no finger) */
        .intr_type    = GPIO_INTR_POSEDGE,     /* Interrupt on rising edge   */
    };
    gpio_config(&io);
    ESP_LOGI(TAG, "TOUCH pin GPIO%d configured (interrupt on rising edge)",
             FP_TOUCH_PIN);
#endif

    ESP_LOGI(TAG, "FP driver init OK");

    /* Detect fingerprint module connection on startup */
    fp_detect_module();

    return ESP_OK;
}


/* ============================================================
 * fp_driver_deinit — Free UART resources
 * ============================================================ */
void fp_driver_deinit(void)
{
    uart_driver_delete(FP_UART_NUM);
    ESP_LOGI(TAG, "FP driver deinitialized");
}

/* ============================================================
 * fp_uart_flush — Discard all bytes waiting in UART RX buffer
 * ============================================================ */
void fp_uart_flush(void)
{
    uart_flush_input(FP_UART_NUM);
}

/* ============================================================
 * fp_uart_read_byte — Read one byte with timeout
 * ============================================================ */
bool fp_uart_read_byte(uint8_t *byte_out, uint32_t timeout_ms)
{
    int got = uart_read_bytes(FP_UART_NUM, byte_out, 1,
                              pdMS_TO_TICKS(timeout_ms));
    return (got == 1);
}

/* ============================================================
 * fp_gen_image — Capture one fingerprint image into sensor buffer
 * ============================================================
 *
 * WHAT HAPPENS INSIDE THE SENSOR:
 *   The R307S has an optical scanner and its own processor.
 *   When GenImg (0x01) is received, the sensor:
 *     1. Activates the optical scanner LED
 *     2. Reads the optical array (256×288 pixels)
 *     3. Stores the 4-bit compressed image in its internal RAM
 *     4. Sends us an ACK with a result code
 *
 *   The image stays in sensor RAM until:
 *     - We call UpImage (to read it out)
 *     - We call GenImg again (it overwrites the buffer)
 *     - Power is lost
 */
int fp_gen_image(void)
{
   // ESP_LOGI(TAG, "GenImg: capturing fingerprint...");

    /* Clear any leftover bytes from previous operations */
    fp_uart_flush();

    /* Send GenImg command: instruction=0x01, no parameters */
    send_command(0x01, NULL, 0);

    /* Wait for ACK packet from sensor */
    fp_packet_t pkt;
    int ret = receive_packet(&pkt, FP_ACK_TIMEOUT_MS);
    if (ret != FP_OK) {
        ESP_LOGW(TAG, "GenImg: ACK timeout or checksum error (err=%d)", ret);
        return ret;
    }

    /* Verify it's an ACK packet (PID = 0x07) */
    if (pkt.pid != FP_PID_ACK) {
        ESP_LOGW(TAG, "GenImg: expected ACK(0x07), got PID=0x%02X", pkt.pid);
        return FP_ERR_SENSOR;
    }

    /* ACK packet always has 1 data byte = result code */
    if (pkt.data_len < 1) {
        ESP_LOGW(TAG, "GenImg: ACK packet has no result code");
        return FP_ERR_SENSOR;
    }

    /* Interpret the result code */
    uint8_t rc = pkt.data[0];
    switch (rc) {
    case FP_RC_OK:
        ESP_LOGI(TAG, "GenImg: SUCCESS — image captured");
        return FP_OK;

    case FP_RC_NO_FINGER:
        /* Not an error — finger just isn't there yet */
        ESP_LOGD(TAG, "GenImg: no finger detected");
        return FP_ERR_NO_FINGER;

    case FP_RC_BAD_IMAGE:
        /* Finger present but image quality is insufficient */
        ESP_LOGW(TAG, "GenImg: finger detected but image quality too low");
        return FP_ERR_BAD_IMAGE;

    default:
        ESP_LOGW(TAG, "GenImg: sensor error code 0x%02X", rc);
        return FP_ERR_SENSOR;
    }
}

/* ============================================================
 * fp_upload_image — Stream the sensor ImageBuffer to ESP32
 * ============================================================
 *
 * DATA FLOW:
 *   Sensor RAM → UART (57600 baud) → ESP32 UART RX buffer
 *             → receive_packet() → callback(data, len, ctx)
 *             → fp_storage_write_chunk() → LittleFS flash
 *
 * PACKET SEQUENCE FROM SENSOR:
 *   1. ACK packet (PID=0x07):  result code 0x00 = "starting transfer"
 *   2–289. Data packets (PID=0x02): each carries 128 bytes of image data
 *          (256 pixels × 4-bit = 128 bytes per row, one packet per row)
 *   290.  End packet (PID=0x08): signals "last chunk" (may also carry data)
 *
 * NOTE: The exact packet count depends on the sensor firmware.
 *       We handle whatever count arrives until PID=0x08.
 */
int fp_upload_image(fp_data_callback_t cb, void *cb_ctx, size_t *bytes_out)
{
    if (!cb) {
        ESP_LOGE(TAG, "fp_upload_image: callback is NULL");
        return FP_ERR_INVALID;
    }

    ESP_LOGI(TAG, "UpImage: requesting image transfer...");

    /* Flush stale bytes before sending command */
    fp_uart_flush();

    /* Send UpImage command: instruction=0x0A, no parameters */
    send_command(0x0A, NULL, 0);

    /* ── STEP 1: Receive and verify the command ACK ── */
    fp_packet_t pkt;
    int ret = receive_packet(&pkt, FP_ACK_TIMEOUT_MS);
    if (ret != FP_OK) {
        ESP_LOGE(TAG, "UpImage: no ACK (err=%d) — sensor may be disconnected", ret);
        return ret;
    }

    if (pkt.pid != FP_PID_ACK) {
        ESP_LOGE(TAG, "UpImage: expected ACK(0x07), got PID=0x%02X", pkt.pid);
        return FP_ERR_SENSOR;
    }

    if (pkt.data_len < 1 || pkt.data[0] != FP_RC_OK) {
        ESP_LOGE(TAG, "UpImage: sensor rejected UpImage, rc=0x%02X",
                 pkt.data_len > 0 ? pkt.data[0] : 0xFF);
        return FP_ERR_SENSOR;
    }

    ESP_LOGI(TAG, "UpImage: ACK OK — sensor beginning image stream...");

    /* ── STEP 2: Receive data packets until end-of-data ── */
    size_t  total_bytes   = 0;
    uint32_t packet_count = 0;

    /* Set overall image transfer deadline */
    int64_t img_deadline = esp_timer_get_time()
                         + (int64_t)FP_IMAGE_TIMEOUT_MS * 1000LL;

    while (esp_timer_get_time() < img_deadline) {

        /* Each packet must arrive within FP_DATA_TIMEOUT_MS */
        ret = receive_packet(&pkt, FP_DATA_TIMEOUT_MS);
        if (ret != FP_OK) {
            ESP_LOGE(TAG, "UpImage: packet %lu error (err=%d, got %zu bytes so far)",
                     (unsigned long)packet_count, ret, total_bytes);
            return ret;
        }

        /* Process based on packet type */
        if (pkt.pid == FP_PID_DATA || pkt.pid == FP_PID_END) {

            /* If there's payload, deliver it to the callback */
            if (pkt.data_len > 0) {
                int cb_ret = cb(pkt.data, pkt.data_len, cb_ctx);
                if (cb_ret != 0) {
                    /* Callback reported an error (e.g., flash write failed) */
                    ESP_LOGE(TAG, "UpImage: callback error %d at packet %lu",
                             cb_ret, (unsigned long)packet_count);
                    return FP_ERR_SENSOR;
                }
                total_bytes += pkt.data_len;
            }

            packet_count++;

            /* Progress log every 32 packets (~4KB) */
            if (packet_count % 32 == 0) {
                ESP_LOGI(TAG, "UpImage progress: %lu packets, %zu bytes",
                         (unsigned long)packet_count, total_bytes);
            }

            if (pkt.pid == FP_PID_END) {
                /* ── TRANSFER COMPLETE ── */
                if (bytes_out) *bytes_out = total_bytes;

                ESP_LOGI(TAG, "UpImage DONE: %lu packets, %zu bytes received",
                         (unsigned long)packet_count, total_bytes);

                /* Sanity check: warn if size differs from expected */
                if (total_bytes != FP_IMAGE_SIZE) {
                    ESP_LOGW(TAG,
                             "UpImage: size mismatch! expected=%d got=%zu "
                             "(sensor firmware variation?)",
                             FP_IMAGE_SIZE, total_bytes);
                }

                return FP_OK;
            }

        } else {
            /* Unexpected PID — log and continue waiting */
            ESP_LOGW(TAG, "UpImage: unexpected PID=0x%02X at packet %lu (skipping)",
                     pkt.pid, (unsigned long)packet_count);
        }
    }

    /* Fell out of loop — image transfer timed out */
    ESP_LOGE(TAG, "UpImage: TIMEOUT — only got %zu of %d bytes (%lu packets)",
             total_bytes, FP_IMAGE_SIZE, (unsigned long)packet_count);
    return FP_ERR_TIMEOUT;
}

/* ============================================================
 * fp_set_led_mode — Attempt to set Aura LED (0x35 command)
 * ============================================================
 * NOTE: Officially only supported by R503. Most R307S modules
 * will reply with 0x0B (Instruction not defined) and ignore this.
 */
int fp_set_led_mode(bool blinking)
{
    /* 0x35 command parameters:
     * ctrl: 0x02 (Blinking) / 0x04 (Always OFF)
     * speed: 0x50 (Blink speed) / 0x00
     * color: 0x01 (Color index)
     * count: 0x00 (Infinite loop)
     */
    uint8_t ctrl  = blinking ? 0x02 : 0x04;
    uint8_t speed = blinking ? 0x50 : 0x00;
    
    uint8_t params[] = { ctrl, speed, 0x01, 0x00 };
    send_command(0x35, params, sizeof(params));

    fp_packet_t resp;
    int err = receive_packet(&resp, FP_ACK_TIMEOUT_MS);
    if (err != FP_OK) return err;

    if (resp.pid != 0x07 || resp.length < 3) {
        return FP_ERR_INVALID;
    }

    uint8_t confirm = resp.data[0];
    if (confirm == 0x00) return FP_OK;
    if (confirm == 0x0B) {
        ESP_LOGD(TAG, "set_led: 0x0B (Instruction Not Defined) - Normal for R307S");
        return FP_ERR_SENSOR;
    }
    
    ESP_LOGW(TAG, "set_led: Sensor returned confirm code 0x%02X", confirm);
    return FP_ERR_SENSOR;
}

/* ============================================================
 * fp_detect_module — Perform handshake to verify module connection
 * ============================================================ */
int fp_detect_module(void)
{
    ESP_LOGI(TAG, "Detecting fingerprint module (Verify Password)...");

    /* Clear any leftover UART bytes */
    fp_uart_flush();

    /* Send Verify Password command: instruction=0x13, parameters={0x00, 0x00, 0x00, 0x00} */
    uint8_t params[] = {0x00, 0x00, 0x00, 0x00};
    send_command(0x13, params, sizeof(params));

    /* Receive the response packet with a 1000ms timeout (generous for handshake) */
    fp_packet_t pkt;
    int ret = receive_packet(&pkt, 1000);
    if (ret != FP_OK) {
        ESP_LOGE(TAG, "Module handshake failed: Timeout or checksum error (err=%d)", ret);
        s_module_detected = false;
        return ret;
    }

    /* Verify it's an ACK packet */
    if (pkt.pid != FP_PID_ACK) {
        ESP_LOGE(TAG, "Module handshake failed: Expected ACK (0x07), got PID=0x%02X", pkt.pid);
        s_module_detected = false;
        return FP_ERR_SENSOR;
    }

    if (pkt.data_len < 1) {
        ESP_LOGE(TAG, "Module handshake failed: Response missing confirmation code");
        s_module_detected = false;
        return FP_ERR_SENSOR;
    }

    uint8_t rc = pkt.data[0];
    if (rc == FP_RC_OK) {
        ESP_LOGI(TAG, "Module handshake SUCCESS: Fingerprint sensor detected!");
        s_module_detected = true;
        return FP_OK;
    }

    ESP_LOGE(TAG, "Module handshake failed: Sensor returned confirm code 0x%02X (Wrong password?)", rc);
    s_module_detected = false;
    return FP_ERR_SENSOR;
}

/* ============================================================
 * fp_is_detected — Return cached connection state
 * ============================================================ */
bool fp_is_detected(void)
{
    return s_module_detected;
}


