/**
 * @file fp_compare.c
 * @brief Layer 5 — RAW Fingerprint Image Similarity Engine.
 *
 * ============================================================
 * THE MATHEMATICS OF IMAGE COMPARISON
 * ============================================================
 *
 * GRAYSCALE PIXEL VALUES:
 *   Each pixel is a 4-bit value (0–15).
 *   0  = black = deep fingerprint ridge valley
 *   15 = white = peak of fingerprint ridge
 *   Two images of the same finger should have similar pixel patterns.
 *
 * STAGE 1 — HISTOGRAM GATE (fast, ~0.5ms):
 * =========================================
 * A histogram counts how many pixels have each brightness value.
 * For a 73,728-pixel image with 16 possible values:
 *   hist[0]  = number of pixels with value 0 (blackest)
 *   hist[1]  = number of pixels with value 1
 *   ...
 *   hist[15] = number of pixels with value 15 (whitest)
 *
 * If two images are from the same finger, their histograms
 * should look similar — similar distributions of bright/dark pixels.
 * If from different fingers, histograms may differ dramatically.
 *
 * BHATTACHARYYA COEFFICIENT:
 *   BC = Σ sqrt(h_a[i] × h_b[i]) / N
 *   where N = total pixels = 73,728
 *
 *   When histograms are identical: BC = N/N = 1.0
 *   When no bin overlaps at all: BC = 0.0
 *   BC < 0.5 → almost certainly different fingers → skip Stage 2
 *
 * STAGE 2 — NORMALISED SAD (main comparison, ~50ms):
 * ===================================================
 * SAD = Sum of Absolute Differences
 *
 * For each pair of corresponding pixels (same row, same column):
 *   diff = |pixel_A - pixel_B|
 *
 * Sum all diffs: total_sad = Σ|pa - pb| across all 73,728 pixel pairs
 *
 * Maximum possible SAD when images are complete opposites:
 *   MAX_SAD = 73,728 × 15 = 1,105,920
 *   (every pixel in A is 0 and in B is 15, or vice versa)
 *
 * Similarity formula:
 *   similarity% = (1 - total_sad / MAX_SAD) × 100
 *
 * Examples:
 *   total_sad = 0        → similarity = 100% (pixel-perfect match)
 *   total_sad = 331,776  → similarity = 70%  (threshold match)
 *   total_sad = 552,960  → similarity = 50%  (uncertain)
 *   total_sad = 1,105,920 → similarity = 0%  (complete opposites)
 *
 * MEMORY OPTIMISATION — ROW-BY-ROW PROCESSING:
 * =============================================
 * Reading full images into RAM would require 2 × 36,864 = 73,728 bytes.
 * Instead, we read one row at a time (128 bytes per file = 256 bytes total)
 * and accumulate the SAD and histogram values.
 * After processing each row, the row buffer is reused for the next.
 *
 * This is possible because SAD is COMMUTATIVE and DECOMPOSABLE:
 *   SAD(A,B) = SAD(A_row1,B_row1) + SAD(A_row2,B_row2) + ...
 *
 * NOTE ON ROTATION/TRANSLATION SENSITIVITY:
 * The SAD metric assumes pixel positions correspond directly.
 * A 3-pixel shift or 2° rotation significantly increases SAD.
 * This is mitigated by storing 5 images at different placements.
 * For production systems, consider adding image normalization.
 * ============================================================
 */
#include "fp_compare.h"
#include "fp_config.h"
#include "fp_storage.h"
#include "esp_log.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

static const char *TAG = "fp_compare";

/* ============================================================
 * fp_compare_files — Compare two RAW image files
 * ============================================================ */
int fp_compare_files(const char *path_a, const char *path_b,
                     uint8_t *similarity_out)
{
    *similarity_out = 0;  /* Default to 0% if anything fails */

    /* ── Open both image files ── */
    FILE *fa = fopen(path_a, "rb");
    if (!fa) {
        ESP_LOGE(TAG, "Cannot open file A: %s", path_a);
        return FP_ERR_STORAGE;
    }

    FILE *fb = fopen(path_b, "rb");
    if (!fb) {
        ESP_LOGE(TAG, "Cannot open file B: %s", path_b);
        fclose(fa);
        return FP_ERR_STORAGE;
    }

    /* ── Row buffers — the only memory needed during comparison ── */
    /* Each row = 256 pixels / 2 pixels-per-byte = 128 bytes       */
    uint8_t row_a[FP_BYTES_PER_ROW];
    uint8_t row_b[FP_BYTES_PER_ROW];

    /* ── Histograms: 16 bins for 4-bit pixels (values 0–15) ──
     * We build BOTH histograms in the same pass as computing SAD.
     * This avoids a second file read for the Bhattacharyya gate. */
    uint32_t hist_a[16] = {0};
    uint32_t hist_b[16] = {0};

    /* ── Running SAD accumulator ──
     * Max SAD = 73,728 × 15 = 1,105,920 — fits comfortably in uint32_t */
    uint32_t total_sad = 0;

    bool io_error = false;

    /* ════════════════════════════════════════════════════════
     * SINGLE PASS: read row by row from both files simultaneously
     * ════════════════════════════════════════════════════════
     * For each of the 288 rows:
     *   1. Read 128 bytes from file A → row_a
     *   2. Read 128 bytes from file B → row_b
     *   3. For each byte: extract 2 nibbles → update hist_a, hist_b, total_sad
     */
    for (int row = 0; row < FP_IMAGE_HEIGHT && !io_error; row++) {

        size_t ra = fread(row_a, 1, FP_BYTES_PER_ROW, fa);
        size_t rb = fread(row_b, 1, FP_BYTES_PER_ROW, fb);

        if (ra != FP_BYTES_PER_ROW || rb != FP_BYTES_PER_ROW) {
            ESP_LOGW(TAG, "Short read at row %d: ra=%zu rb=%zu", row, ra, rb);
            io_error = true;
            break;
        }

        /* Process each byte — it encodes 2 pixels (4-bit packed) */
        for (int i = 0; i < FP_BYTES_PER_ROW; i++) {

            /* Extract two 4-bit pixel values from each packed byte */
            uint8_t p1a = (uint8_t)(row_a[i] >> 4);    /* High nibble, image A */
            uint8_t p2a = (uint8_t)(row_a[i] & 0x0F);  /* Low  nibble, image A */
            uint8_t p1b = (uint8_t)(row_b[i] >> 4);    /* High nibble, image B */
            uint8_t p2b = (uint8_t)(row_b[i] & 0x0F);  /* Low  nibble, image B */

            /* Accumulate histogram counts */
            hist_a[p1a]++;
            hist_a[p2a]++;
            hist_b[p1b]++;
            hist_b[p2b]++;

            /* Accumulate SAD: |pa - pb| for each pixel pair
             * abs(int - int) avoids uint underflow pitfalls */
            total_sad += (uint32_t)abs((int)p1a - (int)p1b);
            total_sad += (uint32_t)abs((int)p2a - (int)p2b);
        }
    }

    fclose(fa);
    fclose(fb);

    if (io_error) {
        ESP_LOGE(TAG, "I/O error during comparison");
        return FP_ERR_STORAGE;
    }

    /* ════════════════════════════════════════════════════════
     * STAGE 1: Histogram gate (Bhattacharyya Coefficient)
     * ════════════════════════════════════════════════════════
     * Compute BC = Σ sqrt(h_a[i] × h_b[i]) / FP_IMAGE_PIXELS
     * ESP32 has an FPU (Floating Point Unit), so sqrtf() is fast.
     */
    float bc = 0.0f;
    for (int i = 0; i < 16; i++) {
        bc += sqrtf((float)hist_a[i] * (float)hist_b[i]);
    }
    bc /= (float)FP_IMAGE_PIXELS;  /* Normalise to 0.0 – 1.0 */

    ESP_LOGD(TAG, "Bhattacharyya BC = %.4f (gate threshold = %.2f)",
             bc, FP_HIST_GATE_THRESHOLD);

    if (bc < FP_HIST_GATE_THRESHOLD) {
        /* Fast reject: histograms are too different.
         * These images are almost certainly from different fingers.
         * Skip the expensive similarity calculation entirely. */
        ESP_LOGD(TAG, "Histogram gate REJECT (BC=%.3f < %.2f)",
                 bc, FP_HIST_GATE_THRESHOLD);
        *similarity_out = 0;
        return FP_OK;
    }

    /* ════════════════════════════════════════════════════════
     * STAGE 2: SAD-based similarity calculation
     * ════════════════════════════════════════════════════════
     * MAX_SAD = all 73,728 pixels having maximum difference (15).
     *         = 73,728 × 15 = 1,105,920
     *
     * similarity = clamp((1 - total_sad / MAX_SAD) × 100, 0, 100)
     */
    const uint32_t MAX_SAD = (uint32_t)FP_IMAGE_PIXELS * 15U;  /* 1,105,920 */

    float sim_f = (1.0f - (float)total_sad / (float)MAX_SAD) * 100.0f;

    /* Clamp to valid range [0, 100] — floating point edge cases */
    if (sim_f < 0.0f)   sim_f = 0.0f;
    if (sim_f > 100.0f) sim_f = 100.0f;

    *similarity_out = (uint8_t)sim_f;

    ESP_LOGI(TAG, "Comparison result: SAD=%lu, BC=%.3f, SIM=%d%%",
             (unsigned long)total_sad, bc, *similarity_out);

    return FP_OK;
}

/* ============================================================
 * fp_compare_best_for_user — Best-of-N match for one user
 * ============================================================
 * Compares the query image against img_count stored images
 * for the given user. Returns the HIGHEST similarity found.
 *
 * RATIONALE:
 * "Best of 5" strategy: even if 4 of the 5 stored images don't
 * match well (different angle), one image that was captured
 * with a similar orientation to the query will produce a high
 * score. Using best-of-N is equivalent to asking:
 * "Has this finger EVER been registered in a similar way?"
 */
int fp_compare_best_for_user(const char *query_path,
                              uint8_t user_id,
                              uint8_t img_count,
                              uint8_t *best_sim_out,
                              uint8_t *best_img_idx_out)
{
    *best_sim_out = 0;
    if (best_img_idx_out) {
        *best_img_idx_out = 1;
    }

    char stored_path[FP_MAX_PATH];

    for (int i = 1; i <= img_count; i++) {
        fp_storage_get_image_path(user_id, (uint8_t)i,
                                  stored_path, sizeof(stored_path));

        uint8_t sim = 0;
        int ret = fp_compare_files(query_path, stored_path, &sim);
        if (ret != FP_OK) {
            /* Log but continue — one bad file shouldn't abort entire search */
            ESP_LOGW(TAG, "compare_best: error comparing img%d for user %d (err=%d)",
                     i, user_id, ret);
            continue;
        }

        ESP_LOGD(TAG, "user=%d img=%d → sim=%d%%", user_id, i, sim);

        if (sim > *best_sim_out) {
            *best_sim_out = sim;
            if (best_img_idx_out) {
                *best_img_idx_out = (uint8_t)i;
            }
        }
    }

    ESP_LOGI(TAG, "Best similarity for user %d: %d%%", user_id, *best_sim_out);
    return FP_OK;
}
