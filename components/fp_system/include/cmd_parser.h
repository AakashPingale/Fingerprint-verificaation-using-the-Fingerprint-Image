/**
 * @file cmd_parser.h
 * @brief Layer 2 — Bluetooth Command Parser and Dispatcher.
 *
 * ============================================================
 * BEGINNER EXPLANATION — WHAT THIS LAYER DOES
 * ============================================================
 * This layer sits between the Bluetooth receive layer and the
 * actual fingerprint/attendance logic.
 *
 * When a command arrives as a raw string like "SET,ID:3",
 * this layer:
 *   1. Tokenises it (splits by comma)
 *   2. Identifies the command keyword (SET, DEL, VERIFY, etc.)
 *   3. Extracts any parameters (ID number)
 *   4. Calls the correct attendance function
 *   5. The attendance function sends results back via bt_send()
 *
 * SUPPORTED COMMANDS:
 * ==================
 *   SET,ID:<n>   Register user n (captures 5 fingerprint images)
 *   DEL,ID:<n>   Delete all data for user n
 *   VERIFY       Capture new image, find best match
 *   LIST         List all registered user IDs
 *   INFO         Show LittleFS flash usage and image count
 *   CLEAR        Delete ALL fingerprint data (use with caution!)
 * ============================================================
 */
#ifndef CMD_PARSER_H
#define CMD_PARSER_H

#include <stdint.h>
#include "esp_err.h"

/* ============================================================
 * Command type enumeration
 * ============================================================
 * An enum gives meaningful names to integer constants.
 * CMD_UNKNOWN handles malformed or unrecognised input gracefully.
 */
typedef enum {
    CMD_UNKNOWN = 0,
    CMD_SET,       /* SET,ID:<n>  — register user n */
    CMD_DEL,       /* DEL,ID:<n>  — delete user n   */
    CMD_VERIFY,    /* VERIFY      — run verification */
    CMD_LIST,      /* LIST        — list users       */
    CMD_INFO,      /* INFO        — flash stats      */
    CMD_CLEAR,     /* CLEAR       — wipe all data    */
} fp_cmd_type_t;

/* ============================================================
 * Parsed command structure
 * ============================================================
 * After parsing, the command and any extracted parameters are
 * stored in this struct for the dispatcher to use.
 */
typedef struct {
    fp_cmd_type_t type;  /* Which command was it?               */
    uint8_t       user_id; /* For SET/DEL: the requested user ID */
} fp_cmd_t;

/* ============================================================
 * PUBLIC API
 * ============================================================ */

/**
 * @brief Parse a raw command string into a fp_cmd_t struct.
 *
 * Does NOT execute anything — only parses.
 *
 * @param raw   Raw command string, e.g. "SET,ID:3"
 * @param cmd   Output struct filled with parsed result
 */
void cmd_parse(const char *raw, fp_cmd_t *cmd);

/**
 * @brief Execute a parsed command by calling the right function.
 *
 * This is the main dispatch function. It calls attendance layer
 * functions and sends results via bt_send().
 *
 * @param cmd  Parsed command struct (from cmd_parse)
 */
void cmd_dispatch(const fp_cmd_t *cmd);

/**
 * @brief Convenience: parse AND dispatch in one call.
 *
 * Typical usage in the main loop:
 *   char line[BT_CMD_MAX_LEN];
 *   if (bt_wait_command(line, sizeof(line), portMAX_DELAY)) {
 *       cmd_process(line);
 *   }
 *
 * @param raw  Raw command string from Bluetooth
 */
void cmd_process(const char *raw);

#endif /* CMD_PARSER_H */
