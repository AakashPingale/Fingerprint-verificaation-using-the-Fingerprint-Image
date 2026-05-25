/**
 * @file fp_debug.c
 * @brief Fingerprint image ASCII-art terminal visualiser (implementation).
 *
 * ============================================================
 * HOW THE ASCII ART PRINTING WORKS — STEP BY STEP
 * ============================================================
 *
 * STEP 1: UNDERSTAND THE PIXEL FORMAT
 * ------------------------------------
 * Each byte in the .raw file contains TWO pixels packed together:
 *
 *   Byte = [P1: bits 7-4] [P2: bits 3-0]
 *
 *   P1 = byte >> 4       (high nibble, value 0-15)
 *   P2 = byte & 0x0F     (low nibble,  value 0-15)
 *
 *   P1 is pixel at column (byte_index × 2)
 *   P2 is pixel at column (byte_index × 2 + 1)
 *
 * STEP 2: MAP PIXEL TO ASCII CHARACTER
 * -------------------------------------
 * The 4-bit grayscale values (0-15) are mapped to ASCII characters.
 * Lower value = darker = ridge (actual fingerprint line).
 * Higher value = lighter = valley (background between ridges).
 *
 *   s_ramp[] = { '@','#','$','%','*','+','=','!',';',':',',','.','\'','`',' ',' ' }
 *   index        0    1    2    3    4    5    6    7    8    9   10   11   12   13  14   15
 *                ←── dark (ridge) ─────────────── light (valley) ───►
 *
 * STEP 3: SUBSAMPLE FOR TERMINAL FIT
 * ------------------------------------
 * 256 pixels wide is too wide for an 80-column terminal.
 * With col_step=4: print 1 out of every 4 pixels → 64 chars wide ✓
 * With row_step=4: print 1 out of every 4 rows   → 72 rows tall  ✓
 *
 * STEP 4: PRINT ROW BY ROW
 * -------------------------
 * - Open the file, read one row at a time (128 bytes)
 * - Skip rows not matching the step (row % row_step != 0)
 * - Build a string of selected pixel characters
 * - Print the line to stdout (→ visible in idf.py monitor)
 * - Close file, print footer
 *
 * MEMORY USAGE:
 * - Row buffer: 128 bytes (one image row)
 * - Line buffer: 260 bytes (one display line + border + null)
 * - TOTAL: ~400 bytes — never loads full 36KB image into RAM ✅
 * ============================================================
 */
#include "fp_debug.h"
#include "fp_config.h"
#include "fp_storage.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <stddef.h>

static const char *TAG = "fp_debug";

/* ============================================================
 * ASCII GRAYSCALE RAMP
 * ============================================================
 * 16 characters, index 0 = darkest (fingerprint ridge),
 *                index 15 = lightest (valley/background).
 *
 * When you look at the terminal output:
 *   Dense '@' '#' '$' characters = fingerprint ridges
 *   Spaces and dots              = valleys between ridges
 *
 * The pattern of ridges IS your fingerprint — each person has
 * a unique arrangement of loops, whorls, and arches.
 */
static const char s_ramp[16] = {
    '@',   /* 0  — very dark ridge center      */
    '#',   /* 1  — dark ridge                  */
    '$',   /* 2  — ridge                       */
    '%',   /* 3  — ridge edge                  */
    '*',   /* 4  — medium                      */
    '+',   /* 5  — medium                      */
    '=',   /* 6  — medium-light                */
    '!',   /* 7  — medium-light                */
    ';',   /* 8  — light                       */
    ':',   /* 9  — light                       */
    ',',   /* 10 — very light                  */
    '.',   /* 11 — very light valley           */
    '\'',  /* 12 — valley                      */
    '`',   /* 13 — deep valley                 */
    ' ',   /* 14 — background                  */
    ' '    /* 15 — deep background             */
};

/* ============================================================
 * Helper: print a horizontal border line of given width
 * ============================================================ */
static void print_border(int width)
{
    putchar('+');
    for (int i = 0; i < width; i++) putchar('-');
    putchar('+');
    printf("\r\n");
}

/* ============================================================
 * fp_debug_print_terminal — Main ASCII art print function
 * ============================================================ */
void fp_debug_print_terminal(const char *raw_path, int col_step, int row_step)
{
    if (!raw_path) return;
    if (col_step < 1) col_step = 1;
    if (row_step < 1) row_step = 1;

    /* Compute display dimensions after subsampling */
    int display_w = FP_IMAGE_WIDTH  / col_step;   /* e.g. 256/4 = 64  */
    int display_h = FP_IMAGE_HEIGHT / row_step;   /* e.g. 288/4 = 72  */

    /* ═══════════════════════════════════════════════
     * PRINT HEADER
     * ═══════════════════════════════════════════════ */
    printf("\r\n");
    print_border(display_w);
    /* Print filename, truncated if too long */
    const char *fname = raw_path;
    /* Find last slash to get just the filename */
    const char *p = raw_path;
    while (*p) { if (*p == '/') fname = p + 1; p++; }
    printf("| FILE: %-*.*s |\r\n",
           display_w - 8, display_w - 8, fname);
    printf("| SIZE: %dx%d  DISPLAY: %dx%d chars (1:%d sub-sample) |\r\n",
           FP_IMAGE_WIDTH, FP_IMAGE_HEIGHT, display_w, display_h, col_step);
    print_border(display_w);

    /* ═══════════════════════════════════════════════
     * OPEN FILE
     * ═══════════════════════════════════════════════ */
    FILE *f = fopen(raw_path, "rb");
    if (!f) {
        printf("| ERROR: Cannot open file!%*s|\r\n",
               display_w - 25, "");
        print_border(display_w);
        ESP_LOGE(TAG, "fp_debug_print_terminal: cannot open '%s'", raw_path);
        return;
    }

    /* ═══════════════════════════════════════════════
     * ROW BUFFER — the only memory used during printing
     * 128 bytes = one complete row of 256 packed pixels
     * ═══════════════════════════════════════════════ */
    uint8_t row_buf[FP_BYTES_PER_ROW];

    /* Line buffer: display_w chars + '|' borders + '\r\n' + null */
    char line_buf[FP_IMAGE_WIDTH + 4];

    int rows_printed = 0;

    for (int row = 0; row < FP_IMAGE_HEIGHT; row++) {

        /* Read one complete row from the .raw file */
        size_t rd = fread(row_buf, 1, FP_BYTES_PER_ROW, f);
        if (rd != FP_BYTES_PER_ROW) {
            ESP_LOGW(TAG, "Short read at row %d (%zu bytes)", row, rd);
            break;
        }

        /* Skip rows we're not displaying (subsampling) */
        if (row % row_step != 0) continue;

        /* ─── Build one display line ─── */
        int line_pos = 0;
        line_buf[line_pos++] = '|';  /* Left border */

        int pixel_col = 0;  /* Tracks which pixel column we're at */

        for (int byte_i = 0; byte_i < FP_BYTES_PER_ROW; byte_i++) {

            /* Unpack the two 4-bit pixels from this byte */
            uint8_t p_high = (uint8_t)(row_buf[byte_i] >> 4);    /* Even pixel (cols 0,2,4,...) */
            uint8_t p_low  = (uint8_t)(row_buf[byte_i] & 0x0F);  /* Odd  pixel (cols 1,3,5,...) */

            /* Even pixel column (byte_i * 2) */
            if (pixel_col % col_step == 0) {
                line_buf[line_pos++] = s_ramp[p_high];
            }
            pixel_col++;

            /* Odd pixel column (byte_i * 2 + 1) */
            if (pixel_col % col_step == 0) {
                line_buf[line_pos++] = s_ramp[p_low];
            }
            pixel_col++;
        }

        line_buf[line_pos++] = '|';  /* Right border */
        line_buf[line_pos++] = '\r';
        line_buf[line_pos++] = '\n';
        line_buf[line_pos]   = '\0';

        /* Write the complete line to stdout in ONE call
         * (avoids mixing printf calls from other tasks) */
        fwrite(line_buf, 1, (size_t)line_pos, stdout);

        rows_printed++;

        /* Yield every 8 printed rows so the Task Watchdog does not
         * trigger during this long print loop.                      */
        if (rows_printed % 8 == 0) {
            vTaskDelay(0);   /* 0 ticks = yield to scheduler, no sleep */
        }
    }

    fclose(f);

    /* ═══════════════════════════════════════════════
     * PRINT FOOTER + LEGEND
     * ═══════════════════════════════════════════════ */
    print_border(display_w);

    /* Print legend explaining the character mapping */
    printf("| LEGEND: @#$%% = ridge (dark)        . , ` ' = valley (light) |\r\n");
    printf("|         Ridges form the fingerprint pattern                   |\r\n");
    print_border(display_w);
    printf("\r\n");

    ESP_LOGI(TAG, "Printed image: %s (%d rows shown, display %dx%d)",
             raw_path, rows_printed, display_w, display_h);
}

/* ============================================================
 * fp_debug_print_user_images — Print all images for a user
 * ============================================================ */
void fp_debug_print_user_images(uint8_t user_id, uint8_t img_count)
{
    char path[FP_MAX_PATH];

    ESP_LOGI(TAG, "=== Printing %d images for user %d ===", img_count, user_id);

    for (uint8_t i = 1; i <= img_count; i++) {

        fp_storage_get_image_path(user_id, i, path, sizeof(path));

        printf("\r\n");
        printf("##############################################\r\n");
        printf("# USER %d — IMAGE %d / %d                    #\r\n",
               user_id, i, img_count);
        printf("##############################################\r\n");

        fp_debug_print_terminal(path, FP_DBG_COL_STEP, FP_DBG_ROW_STEP);

        /* Small delay so the terminal doesn't get overwhelmed */
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    ESP_LOGI(TAG, "=== Done printing user %d images ===", user_id);
}

/* ============================================================
 * fp_debug_compare_visual — Side-by-side comparison in terminal
 * ============================================================
 *
 * Prints two images side-by-side using half-width subsampling.
 * Each image uses col_step=8 (32 chars wide), separated by ' | '.
 * Total display width: 32 + 3 + 32 = 67 chars.
 *
 * EXAMPLE:
 *   +--------------------------------+  +--------------------------------+
 *   |   STORED: /fp/.../1/fp1.raw    |  | QUERY: /fp/.../tmp/verify.raw  |
 *   |@@#.    .;@@@@;.    .#@@;      |  |@@#.    .;@@@@:.    .#@@;       |
 *   |  @@@#$%*+====+*%$#@@@         |  |  @@@#$%*+====+*%$#@@@          |
 *   +--------------------------------+  +--------------------------------+
 *   Similarity: 82%  [MATCH]
 */
void fp_debug_compare_visual(const char *stored_path, const char *query_path,
                              uint8_t similarity)
{
    /* Use half-width for each image so both fit side-by-side */
    const int half_step = FP_DBG_COL_STEP * 2;  /* 8 → 32 chars per image */
    const int half_w    = FP_IMAGE_WIDTH / half_step;  /* 32 */

    FILE *fa = fopen(stored_path, "rb");
    FILE *fb = fopen(query_path,  "rb");

    if (!fa || !fb) {
        ESP_LOGW(TAG, "compare_visual: cannot open one or both files");
        if (fa) fclose(fa);
        if (fb) fclose(fb);
        /* Fall back to printing each separately */
        fp_debug_print_terminal(stored_path, FP_DBG_COL_STEP, FP_DBG_ROW_STEP);
        fp_debug_print_terminal(query_path,  FP_DBG_COL_STEP, FP_DBG_ROW_STEP);
        return;
    }

    /* Header */
    printf("\r\n");
    printf("+%-*.*s+  +%-*.*s+\r\n",
           half_w, half_w, "---STORED---",
           half_w, half_w, "---QUERY----");
    printf("|%-*.*s|  |%-*.*s|\r\n",
           half_w, half_w, stored_path,
           half_w, half_w, query_path);
    printf("+");
    for (int i=0;i<half_w;i++) putchar('-');
    printf("+  +");
    for (int i=0;i<half_w;i++) putchar('-');
    printf("+\r\n");

    uint8_t row_a[FP_BYTES_PER_ROW];
    uint8_t row_b[FP_BYTES_PER_ROW];
    char    line_a[128];
    char    line_b[128];

    for (int row = 0; row < FP_IMAGE_HEIGHT; row++) {
        size_t ra = fread(row_a, 1, FP_BYTES_PER_ROW, fa);
        size_t rb = fread(row_b, 1, FP_BYTES_PER_ROW, fb);
        if (ra != FP_BYTES_PER_ROW || rb != FP_BYTES_PER_ROW) break;

        if (row % (FP_DBG_ROW_STEP * 2) != 0) continue;  /* 2× row subsampling */

        /* Build both half-width lines */
        int pa = 0, pb = 0;
        int col_a = 0, col_b = 0;

        for (int byte_i = 0; byte_i < FP_BYTES_PER_ROW; byte_i++) {
            uint8_t pha = row_a[byte_i] >> 4;  uint8_t pla = row_a[byte_i] & 0x0F;
            uint8_t phb = row_b[byte_i] >> 4;  uint8_t plb = row_b[byte_i] & 0x0F;

            if (col_a % half_step == 0 && pa < half_w) line_a[pa++] = s_ramp[pha];
            col_a++;
            if (col_a % half_step == 0 && pa < half_w) line_a[pa++] = s_ramp[pla];
            col_a++;

            if (col_b % half_step == 0 && pb < half_w) line_b[pb++] = s_ramp[phb];
            col_b++;
            if (col_b % half_step == 0 && pb < half_w) line_b[pb++] = s_ramp[plb];
            col_b++;
        }
        line_a[pa] = '\0';
        line_b[pb] = '\0';

        printf("|%-*s|  |%-*s|\r\n", half_w, line_a, half_w, line_b);
    }

    fclose(fa);
    fclose(fb);

    /* Footer with similarity result */
    printf("+");
    for (int i=0;i<half_w;i++) putchar('-');
    printf("+  +");
    for (int i=0;i<half_w;i++) putchar('-');
    printf("+\r\n");

    if (similarity >= FP_DEFAULT_THRESHOLD) {
        printf(">>> Similarity: %d%%  [=== MATCH ===]  (threshold: %d%%)\r\n",
               similarity, FP_DEFAULT_THRESHOLD);
    } else {
        printf(">>> Similarity: %d%%  [--- NO MATCH ---]  (threshold: %d%%)\r\n",
               similarity, FP_DEFAULT_THRESHOLD);
    }
    printf("\r\n");
}

/* ============================================================
 * fp_debug_dump_hex — Dump raw image as continuous HEX
 * ============================================================ */
void fp_debug_dump_hex(const char *raw_path)
{
    FILE *f = fopen(raw_path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "dump_hex: cannot open %s", raw_path);
        return;
    }

    printf("\r\n--- BEGIN RAW IMAGE HEX DUMP: %s ---\r\n", raw_path);
    
    /* 32 bytes per line = 64 hex characters */
    uint8_t buf[32];
    size_t rd;
    int total = 0;
    
    while ((rd = fread(buf, 1, sizeof(buf), f)) > 0) {
        for (size_t i = 0; i < rd; i++) {
            printf("%02X", buf[i]);
        }
        printf("\r\n");
        total += rd;
    }
    
    printf("--- END RAW IMAGE HEX DUMP (%d bytes) ---\r\n\r\n", total);
    fclose(f);
}

/* ============================================================
 * fp_debug_print_base64 — Print image file as Base64 string
 * ============================================================ */
static const char s_b64_table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

void fp_debug_print_base64(const char *file_path)
{
    FILE *f = fopen(file_path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "print_base64: cannot open %s", file_path);
        return;
    }

    ESP_LOGI(TAG, "--- BEGIN BASE64: %s ---", file_path);
    printf("\r\n");

    uint8_t in[3];
    char out[5];
    out[4] = '\0';
    size_t rd;
    int col = 0;

    while ((rd = fread(in, 1, 3, f)) > 0) {
        out[0] = s_b64_table[in[0] >> 2];
        out[1] = s_b64_table[((in[0] & 0x03) << 4) | (rd > 1 ? (in[1] >> 4) : 0)];
        out[2] = rd > 1 ? s_b64_table[((in[1] & 0x0F) << 2) | (rd > 2 ? (in[2] >> 6) : 0)] : '=';
        out[3] = rd > 2 ? s_b64_table[in[2] & 0x3F] : '=';
        
        printf("%s", out);
        col += 4;
        if (col >= 76) {
            printf("\r\n");
            col = 0;
            /* Yield periodically so watchdog doesn't trigger */
            vTaskDelay(0);
        }
    }
    printf("\r\n--- END BASE64 ---\r\n");
    fclose(f);
}

/* ============================================================
 * fp_debug_print_bmp_base64 — Print .raw as a Base64 BMP image
 * ============================================================ */
void fp_debug_print_bmp_base64(const char *raw_path)
{
    FILE *f = fopen(raw_path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "print_bmp_base64: cannot open %s", raw_path);
        return;
    }

    ESP_LOGI(TAG, "--- BEGIN BMP BASE64: %s ---", raw_path);
    printf("\r\n");

    /* Construct 118-byte BMP Header for 256x288 4-bit Grayscale */
    const uint8_t bmp_header[118] = {
        /* BITMAPFILEHEADER (14 bytes) */
        'B', 'M',               /* Magic */
        0x76, 0x90, 0x00, 0x00, /* File size (118 + 36864 = 36982 = 0x9076) */
        0x00, 0x00,             /* Reserved1 */
        0x00, 0x00,             /* Reserved2 */
        0x76, 0x00, 0x00, 0x00, /* Pixel data offset (118) */

        /* BITMAPINFOHEADER (40 bytes) */
        0x28, 0x00, 0x00, 0x00, /* Header size (40) */
        0x00, 0x01, 0x00, 0x00, /* Width (256 = 0x0100) */
        0xE0, 0xED, 0xFF, 0xFF, /* Height (-288 = 0xFFFFEDE0) Top-down */
        0x01, 0x00,             /* Planes (1) */
        0x04, 0x00,             /* BPP (4-bit) */
        0x00, 0x00, 0x00, 0x00, /* Compression (None) */
        0x00, 0x90, 0x00, 0x00, /* Image size (36864 = 0x9000) */
        0x13, 0x0B, 0x00, 0x00, /* X pixels/m (2835) */
        0x13, 0x0B, 0x00, 0x00, /* Y pixels/m (2835) */
        0x10, 0x00, 0x00, 0x00, /* Colors (16) */
        0x10, 0x00, 0x00, 0x00, /* Important Colors (16) */

        /* PALETTE (16 colors * 4 bytes = 64 bytes) */
        0x00,0x00,0x00,0x00, 0x11,0x11,0x11,0x00, 0x22,0x22,0x22,0x00, 0x33,0x33,0x33,0x00,
        0x44,0x44,0x44,0x00, 0x55,0x55,0x55,0x00, 0x66,0x66,0x66,0x00, 0x77,0x77,0x77,0x00,
        0x88,0x88,0x88,0x00, 0x99,0x99,0x99,0x00, 0xAA,0xAA,0xAA,0x00, 0xBB,0xBB,0xBB,0x00,
        0xCC,0xCC,0xCC,0x00, 0xDD,0xDD,0xDD,0x00, 0xEE,0xEE,0xEE,0x00, 0xFF,0xFF,0xFF,0x00
    };

    uint8_t in[3];
    char out[5];
    out[4] = '\0';
    int col = 0;
    size_t header_idx = 0;
    bool header_done = false;

    while (true) {
        int rd = 0;
        /* Seamlessly buffer 3 bytes from header, then from file */
        while (rd < 3) {
            if (!header_done) {
                in[rd++] = bmp_header[header_idx++];
                if (header_idx >= sizeof(bmp_header)) {
                    header_done = true;
                }
            } else {
                if (fread(&in[rd], 1, 1, f) == 0) break;
                rd++;
            }
        }

        if (rd == 0) break;

        out[0] = s_b64_table[in[0] >> 2];
        out[1] = s_b64_table[((in[0] & 0x03) << 4) | (rd > 1 ? (in[1] >> 4) : 0)];
        out[2] = rd > 1 ? s_b64_table[((in[1] & 0x0F) << 2) | (rd > 2 ? (in[2] >> 6) : 0)] : '=';
        out[3] = rd > 2 ? s_b64_table[in[2] & 0x3F] : '=';
        
        printf("%s", out);
        col += 4;
        if (col >= 76) {
            printf("\r\n");
            col = 0;
            vTaskDelay(0); /* Yield */
        }

        if (rd < 3) break;
    }

    printf("\r\n--- END BMP BASE64 ---\r\n");
    fclose(f);
}
