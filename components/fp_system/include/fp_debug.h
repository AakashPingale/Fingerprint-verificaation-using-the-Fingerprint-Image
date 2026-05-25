/**
 * @file fp_debug.h
 * @brief Fingerprint image ASCII-art terminal visualiser.
 *
 * ============================================================
 * WHAT THIS MODULE DOES
 * ============================================================
 * When debugging a fingerprint system, it is extremely useful
 * to SEE the captured image without needing a PC tool.
 *
 * This module converts the RAW 4-bit pixel data stored in a
 * .raw file into ASCII art and prints it directly to the
 * ESP32 serial console — visible in "idf.py monitor".
 *
 * HOW DOES IT WORK?
 * =================
 * 1. Read the .raw file row by row (128 bytes per row)
 * 2. Unpack each byte into 2 pixels (high nibble + low nibble)
 * 3. Map each pixel value (0–15) to an ASCII character
 *    - Value 0  (darkest = ridge) → '@'
 *    - Value 15 (lightest = valley) → ' '
 * 4. Print every Nth row and every Nth column (subsampling)
 *    to fit the image into terminal width
 *
 * EXAMPLE OUTPUT (col_step=4, row_step=4 → 64×72 chars):
 * =========================================================
 * +----------------------------------------------------------------+
 * |                         .,:;;;;;:,.                           |
 * |                       .;@@@@@@@@@@@@;.                        |
 * |                      ;@@#$$$$$$$$#@@;                         |
 * |                     ;@@#$%****%$#@@;                          |
 * |                    ;@@#$%*+==+*%$#@@;                         |
 * |                     ;@@#$%****%$#@@;                          |
 * |                      ;@@#$$$$$$$$#@@;                         |
 * +----------------------------------------------------------------+
 * Legend: @#$%*+ = ridge (dark)   .:, ` = valley (light)
 *
 * PIXEL MAPPING (16 levels):
 * ==========================
 *   0  →  @   (darkest — fingerprint ridge core)
 *   1  →  #
 *   2  →  $
 *   3  →  %
 *   4  →  *
 *   5  →  +
 *   6  →  =
 *   7  →  !
 *   8  →  ;
 *   9  →  :
 *   10 →  ,
 *   11 →  .
 *   12 →  '
 *   13 →  `
 *   14 →  (space)
 *   15 →  (space)  (lightest — valley/background)
 *
 * SUBSAMPLING:
 * ============
 * A 256×288 image displayed 1:1 would require 256 columns
 * and 288 rows — far too large for a terminal.
 * By printing every 4th column and every 4th row, we get a
 * 64×72 character display that fits in an 80-column terminal.
 *
 * col_step=4, row_step=4  →  64  wide × 72  rows (recommended)
 * col_step=3, row_step=3  →  85  wide × 96  rows (more detail)
 * col_step=2, row_step=2  →  128 wide × 144 rows (high detail)
 * ============================================================
 */
#ifndef FP_DEBUG_H
#define FP_DEBUG_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * DEFAULT DISPLAY PARAMETERS
 * ============================================================ */
#define FP_DBG_COL_STEP   4   /* Print every 4th column → 64 chars wide  */
#define FP_DBG_ROW_STEP   4   /* Print every 4th row    → 72 rows tall   */

/* ============================================================
 * PUBLIC API
 * ============================================================ */

/**
 * @brief Print a RAW fingerprint image as ASCII art to the serial console.
 *
 * Output appears in "idf.py monitor" terminal.
 * Uses printf() which maps to UART0 (the debug console UART).
 *
 * Memory usage: only FP_BYTES_PER_ROW (128 bytes) + one char-line buffer.
 * No full image ever loaded into RAM.
 *
 * @param raw_path   Full path to .raw file, e.g. "/fp/fingerprints/1/fp2.raw"
 * @param col_step   Subsample columns (4 = print every 4th pixel column)
 * @param row_step   Subsample rows    (4 = print every 4th pixel row)
 */
void fp_debug_print_terminal(const char *raw_path, int col_step, int row_step);

/**
 * @brief Print all stored images for one user to the serial console.
 *
 * Prints images 1 through img_count with clear headers between them.
 * A short delay separates each image for readability.
 *
 * @param user_id    User whose images to print
 * @param img_count  How many images to print (from meta.txt)
 */
void fp_debug_print_user_images(uint8_t user_id, uint8_t img_count);

/**
 * @brief Print both a stored image and the current temp verify image side-by-side.
 *
 * Useful for visually confirming whether two fingerprints look similar.
 * Side-by-side comparison in terminal: 32+32 = 64 chars wide.
 *
 * @param stored_path   Path to the stored image
 * @param query_path    Path to the query (verify) image
 * @param similarity    Similarity % to show in header
 */
void fp_debug_compare_visual(const char *stored_path, const char *query_path,
                              uint8_t similarity);

/**
 * @brief Dump the raw grayscale image data as HEX strings to the serial terminal.
 *
 * Useful for copying the raw image data from the terminal to send to a server later.
 * Format: Continuous HEX string, 32 bytes (64 hex characters) per line.
 *
 * @param raw_path   Path to the .raw file
 */
void fp_debug_dump_hex(const char *raw_path);

/**
 * @brief Print the contents of a file as a Base64 string to the terminal.
 * Useful for exporting images via serial monitor.
 */
void fp_debug_print_base64(const char *file_path);

/**
 * @brief Print the raw image file as a Base64 encoded BMP image.
 * This prepends a 4-bit grayscale BMP header so it can be viewed in a browser.
 */
void fp_debug_print_bmp_base64(const char *raw_path);

#endif /* FP_DEBUG_H */
