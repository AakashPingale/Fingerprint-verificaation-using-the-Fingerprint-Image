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

/* Helper to shift an array of bytes left or right (bit shift) */
static void shift_byte_array(const uint8_t *src, uint8_t *dst, size_t len, int shift)
{
    if (shift == 0) {
        memcpy(dst, src, len);
        return;
    }
    memset(dst, 0, len);
    if (shift > 0) { /* Left shift */
        for (size_t i = 0; i < len; i++) {
            dst[i] = src[i] << shift;
            if (i < len - 1) {
                dst[i] |= src[i + 1] >> (8 - shift);
            }
        }
    } else { /* Right shift, shift is negative */
        int rshift = -shift;
        for (size_t i = 0; i < len; i++) {
            dst[i] = src[i] >> rshift;
            if (i > 0) {
                dst[i] |= src[i - 1] << (8 - rshift);
            }
        }
    }
}

/* ============================================================
 * fp_compare_files — Compare two BINARY image files
 * ============================================================ */
int fp_compare_files(const char *bin_path_a, const char *bin_path_b,
                     uint8_t *similarity_out)
{
    *similarity_out = 0;

    FILE *fa = fopen(bin_path_a, "rb");
    FILE *fb = fopen(bin_path_b, "rb");
    if (!fa || !fb) {
        if (fa) fclose(fa);
        if (fb) fclose(fb);
        return FP_ERR_STORAGE;
    }

    /* Step 2 — Skip rows before FP_ROI_START_ROW */
    fseek(fa, FP_ROI_START_ROW * FP_BIN_ROW_BYTES, SEEK_SET);
    fseek(fb, FP_ROI_START_ROW * FP_BIN_ROW_BYTES, SEEK_SET);

    uint8_t row_a[FP_BIN_ROW_BYTES];
    uint8_t row_b[FP_BIN_ROW_BYTES];
    uint32_t total_matching_bits = 0;

    /* Step 3 — For each row in ROI */
    for (int row = FP_ROI_START_ROW; row < FP_ROI_END_ROW; row++) {
        if (fread(row_a, 1, FP_BIN_ROW_BYTES, fa) != FP_BIN_ROW_BYTES) break;
        if (fread(row_b, 1, FP_BIN_ROW_BYTES, fb) != FP_BIN_ROW_BYTES) break;

        /* Extract only the ROI column bytes (bytes 6-25) */
        uint8_t roi_a[FP_ROI_ROW_BYTES];
        uint8_t roi_b[FP_ROI_ROW_BYTES];
        memcpy(roi_a, &row_a[FP_ROI_START_COL / 8], FP_ROI_ROW_BYTES);
        memcpy(roi_b, &row_b[FP_ROI_START_COL / 8], FP_ROI_ROW_BYTES);

        uint32_t best_shift_score = 0;

        /* Horizontal bit-shift search */
        for (int shift = -3; shift <= 3; shift++) {
            uint8_t shifted_b[FP_ROI_ROW_BYTES];
            shift_byte_array(roi_b, shifted_b, FP_ROI_ROW_BYTES, shift);

            uint32_t differing_bits = 0;
            for (int i = 0; i < FP_ROI_ROW_BYTES; i++) {
                uint8_t xor_val = roi_a[i] ^ shifted_b[i];
                differing_bits += __builtin_popcount(xor_val);
            }

            uint32_t matching_bits_this_shift = (FP_ROI_ROW_BYTES * 8) - differing_bits;
            if (matching_bits_this_shift > best_shift_score) {
                best_shift_score = matching_bits_this_shift;
            }
        }
        total_matching_bits += best_shift_score;
    }

    fclose(fa);
    fclose(fb);

    /* Step 4 — Calculate final similarity percentage */
    uint32_t total_roi_bits = (FP_ROI_END_ROW - FP_ROI_START_ROW) * (FP_ROI_ROW_BYTES * 8);
    if (total_roi_bits > 0) {
        *similarity_out = (uint8_t)((total_matching_bits * 100) / total_roi_bits);
    }
    
    ESP_LOGI(TAG, "Binary Comparison result: SIM=%d%%", *similarity_out);

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
        fp_storage_get_bin_path(user_id, (uint8_t)i,
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
