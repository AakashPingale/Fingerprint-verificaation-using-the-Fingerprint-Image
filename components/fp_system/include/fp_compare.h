/**
 * @file fp_compare.h
 * @brief Layer 5 — RAW Fingerprint Image Similarity Comparison.
 *
 * ============================================================
 * BEGINNER EXPLANATION — IMAGE-BASED MATCHING
 * ============================================================
 * This is the core "biometric engine" of our system.
 * Instead of using the R307S sensor's proprietary template
 * matching (Img2Tz + Match/Search), we compare images ourselves.
 *
 * WHY IMAGES DIFFER (even same finger):
 *   Each time you place your finger on a sensor, the image
 *   is slightly different due to:
 *   • TRANSLATION: finger placed 2mm left/right/up/down
 *   • ROTATION:    finger tilted 5° clockwise
 *   • PRESSURE:    pressing hard spreads ridges, light press tightens them
 *   • MOISTURE:    wet/dry skin reflects light differently
 *   • SENSOR NOISE: random 1-2 pixel level variation
 *
 *   Because of this, we can NEVER expect two images to match 100%.
 *   We use a THRESHOLD: if similarity ≥ 70%, we declare a match.
 *
 * OUR ALGORITHM — TWO STAGE:
 * ===========================
 *
 * STAGE 1 — Histogram Correlation (fast pre-filter, ~0.5ms):
 *   Build a histogram (count of each pixel value 0-15) for
 *   both images. Compare histograms using Bhattacharyya distance.
 *   If histograms are very different, the images are definitely
 *   from different fingers — reject immediately without doing
 *   the expensive pixel comparison.
 *
 *   Bhattacharyya Coefficient (BC):
 *     BC = Σ sqrt(H_a[i] × H_b[i]) / total_pixels
 *   BC = 1.0 → identical histograms
 *   BC = 0.0 → completely different histograms
 *   We fast-reject if BC < 0.5 (FP_HIST_GATE_THRESHOLD)
 *
 * STAGE 2 — SAD Pixel Similarity (full comparison, ~50ms):
 *   SAD = Sum of Absolute Differences
 *   For every pair of corresponding pixels, compute |pA - pB|
 *   Sum all differences → total_sad
 *
 *   Maximum possible SAD = 73,728 pixels × 15 (max difference) = 1,105,920
 *   Similarity % = (1 - total_sad / 1,105,920) × 100
 *
 *   IMPLEMENTATION: Reads both files row by row (128 bytes each).
 *   Only 256 bytes of RAM used at any time. ✅
 *
 * MEMORY USAGE DURING COMPARISON:
 *   row_a buffer:     128 bytes
 *   row_b buffer:     128 bytes
 *   hist_a[16]:        64 bytes (16 × 4-byte uint32)
 *   hist_b[16]:        64 bytes
 *   running total_sad:  4 bytes
 *   TOTAL:            ~400 bytes  (vs 36KB+ if we loaded full images)
 *
 * LIMITATIONS OF THIS APPROACH:
 *   • Sensitive to rotation: a 5° rotation misaligns pixels significantly
 *   • Sensitive to translation: shifted images have poor pixel alignment
 *   • Not scale-invariant: different pressure changes ridge width
 *   → MITIGATION: storing 5 images per user covers common variations
 *   → FUTURE UPGRADE: implement image normalization (alignment) before SAD
 * ============================================================
 */
#ifndef FP_COMPARE_H
#define FP_COMPARE_H

#include <stdint.h>
#include "fp_driver.h"   /* For error codes */

/* ============================================================
 * PUBLIC API
 * ============================================================ */

/**
 * @brief Compare two RAW fingerprint image files.
 *
 * Implements two-stage comparison:
 *   Stage 1: Histogram Bhattacharyya gate (fast reject)
 *   Stage 2: Row-by-row SAD similarity calculation
 *
 * Both files must be exactly FP_IMAGE_SIZE (36,864) bytes.
 * Files are read row by row — full image NEVER loaded into RAM.
 *
 * EXAMPLE:
 *   uint8_t sim;
 *   fp_compare_files("/fp/fingerprints/tmp/verify.raw",
 *                    "/fp/fingerprints/1/fp3.raw",
 *                    &sim);
 *   if (sim >= 70) { // MATCH! }
 *
 * @param path_a         Full path to first image file (e.g., new capture)
 * @param path_b         Full path to second image file (e.g., stored)
 * @param similarity_out Output: similarity percentage 0–100
 * @return FP_OK on success, FP_ERR_STORAGE on file open/read failure
 */
int fp_compare_files(const char *path_a, const char *path_b,
                     uint8_t *similarity_out);

/**
 * @brief Find the best match for a captured image across all stored images.
 *
 * Compares the given image file against ALL stored images for ONE user.
 * Returns the highest similarity score found among the user's images.
 *
 * This is called once per registered user during VERIFY.
 *
 * @param query_path    Path to the captured image to match
 * @param user_id       Which user's stored images to compare against
 * @param img_count     How many images this user has (from meta.txt)
 * @param best_sim_out  Output: highest similarity % found
 * @return FP_OK on success
 */
int fp_compare_best_for_user(const char *query_path, uint8_t user_id,
                              uint8_t img_count, uint8_t *best_sim_out,
                              uint8_t *best_img_idx_out);

#endif /* FP_COMPARE_H */
