/**
 * @file cmd_parser.c
 * @brief Layer 2 — Command Parser and Dispatcher Implementation.
 *
 * ============================================================
 * HOW COMMAND PARSING WORKS
 * ============================================================
 * We receive raw text strings like:
 *   "SET,ID:3"
 *   "VERIFY"
 *   "DEL,ID:1"
 *
 * PARSING STEPS:
 *   1. Convert to uppercase (so "set,id:1" also works)
 *   2. Tokenise by comma → ["SET", "ID:3"]
 *   3. Compare first token to known command names
 *   4. If command has parameters, parse them from second token
 *   5. Fill fp_cmd_t struct with results
 *   6. Call appropriate attendance_* function
 *
 * DESIGN RATIONALE:
 *   Keeping parsing separate from execution (parse → then dispatch)
 *   makes testing easier and keeps functions small/focused.
 * ============================================================
 */
#include "cmd_parser.h"
#include "attendance.h"
#include "bt_serial.h"
#include "fp_config.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>

static const char *TAG = "cmd_parser";

/* ============================================================
 * Helper: convert string to uppercase in-place
 * ============================================================ */
static void str_upper(char *s)
{
    while (*s) {
        *s = (char)toupper((unsigned char)*s);
        s++;
    }
}

/* ============================================================
 * cmd_parse — Parse a raw command string into fp_cmd_t
 * ============================================================ */
void cmd_parse(const char *raw, fp_cmd_t *cmd)
{
    /* Initialize to unknown state */
    cmd->type    = CMD_UNKNOWN;
    cmd->user_id = 0;

    /* Work on a mutable copy — strtok modifies the string */
    char buf[BT_CMD_MAX_LEN];
    strncpy(buf, raw, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    /* Convert to uppercase for case-insensitive matching */
    str_upper(buf);

    /* Tokenise by comma: first token = command keyword */
    char *token = strtok(buf, ",");
    if (!token) return;

    /* --- Match command keyword --- */

    if (strcmp(token, "SET") == 0) {
        /* SET,ID:<n> — Parse the user ID from second token */
        cmd->type = CMD_SET;
        char *id_token = strtok(NULL, ",");
        if (id_token && strncmp(id_token, "ID:", 3) == 0) {
            cmd->user_id = (uint8_t)atoi(id_token + 3);
            if (cmd->user_id == 0) {
                /* Invalid user ID */
                ESP_LOGW(TAG, "SET: invalid user ID in '%s'", raw);
                cmd->type = CMD_UNKNOWN;
            }
        } else {
            ESP_LOGW(TAG, "SET: missing ID parameter in '%s'", raw);
            cmd->type = CMD_UNKNOWN;
        }

    } else if (strcmp(token, "DEL") == 0) {
        /* DEL,ID:<n> */
        cmd->type = CMD_DEL;
        char *id_token = strtok(NULL, ",");
        if (id_token && strncmp(id_token, "ID:", 3) == 0) {
            cmd->user_id = (uint8_t)atoi(id_token + 3);
            if (cmd->user_id == 0) {
                ESP_LOGW(TAG, "DEL: invalid user ID in '%s'", raw);
                cmd->type = CMD_UNKNOWN;
            }
        } else {
            ESP_LOGW(TAG, "DEL: missing ID parameter in '%s'", raw);
            cmd->type = CMD_UNKNOWN;
        }

    } else if (strcmp(token, "VERIFY") == 0) {
        cmd->type = CMD_VERIFY;

    } else if (strcmp(token, "LIST") == 0) {
        cmd->type = CMD_LIST;

    } else if (strcmp(token, "INFO") == 0) {
        cmd->type = CMD_INFO;

    } else if (strcmp(token, "CLEAR") == 0) {
        cmd->type = CMD_CLEAR;

    } else {
        ESP_LOGW(TAG, "Unknown command: '%s'", raw);
        cmd->type = CMD_UNKNOWN;
    }
}

/* ============================================================
 * cmd_dispatch — Execute a parsed command
 * ============================================================ */
void cmd_dispatch(const fp_cmd_t *cmd)
{
    switch (cmd->type) {

    case CMD_SET:
        ESP_LOGI(TAG, "Dispatching SET for user %d", cmd->user_id);
        attendance_register_user(cmd->user_id);
        break;

    case CMD_DEL:
        ESP_LOGI(TAG, "Dispatching DEL for user %d", cmd->user_id);
        attendance_delete_user(cmd->user_id);
        break;

    case CMD_VERIFY:
        ESP_LOGI(TAG, "Dispatching VERIFY");
        attendance_verify();
        break;

    case CMD_LIST:
        ESP_LOGI(TAG, "Dispatching LIST");
        attendance_list();
        break;

    case CMD_INFO:
        ESP_LOGI(TAG, "Dispatching INFO");
        attendance_info();
        break;

    case CMD_CLEAR:
        ESP_LOGI(TAG, "Dispatching CLEAR");
        attendance_clear();
        break;

    case CMD_UNKNOWN:
    default:
        bt_send("ERR,UNKNOWN_CMD\r\n");
        bt_send("Commands: SET,ID:<n> | DEL,ID:<n> | VERIFY | LIST | INFO | CLEAR\r\n");
        break;
    }

    /* Print command prompt after every response */
    bt_send("> ");
}

/* ============================================================
 * cmd_process — Parse AND dispatch in one convenience call
 * ============================================================ */
void cmd_process(const char *raw)
{
    ESP_LOGI(TAG, "Processing: '%s'", raw);

    fp_cmd_t cmd;
    cmd_parse(raw, &cmd);
    cmd_dispatch(&cmd);
}
