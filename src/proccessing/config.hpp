/**
 * @file config.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Project-wide scalar type and processing pipeline constants.
 * @date 2026-05-14
 */

#ifndef CONFIG_HPP
#define CONFIG_HPP

namespace CFG
{

/// @brief Floating-point precision used throughout the processing pipeline.
typedef double Scalar;

/// @brief Bandpass highpass corner in Hz. Rejects sub-swell drift and DC.
constexpr Scalar bandpass_lo_hz = 0.05;

/// @brief Bandpass lowpass corner in Hz. Upper edge of the ocean wave band.
constexpr Scalar bandpass_hi_hz = 0.5;

/**
 * @brief Duration of each Welch analysis window in seconds.
 *
 * Sea state is treated as stationary over this interval, following standard
 * oceanographic practice (CDIP, NDBC use 17-20 minute records). Adjacent
 * windows overlap by 50%.
 */
constexpr Scalar analysis_window_s = 1200.0;

/**
 * @brief DFT length and samples per Welch segment.
 *
 * Override at configure time with -DSMARTFIN_WELCH_NPERSEG=<N>.
 * Must be a power of 2 and satisfy nperseg >= 4.
 * Default (256) at a 5 Hz decimated rate gives df = 0.0195 Hz.
 */
#ifndef SMARTFIN_WELCH_NPERSEG
#define SMARTFIN_WELCH_NPERSEG 256
#endif

constexpr int welch_nperseg = SMARTFIN_WELCH_NPERSEG;
static_assert((welch_nperseg > 0 && (welch_nperseg & (welch_nperseg - 1)) == 0),
              "welch_nperseg must be a power of 2!");
/**
 * @brief Overlap between adjacent Welch segments in samples.
 *
 * Fixed at 50% of nperseg. Satisfies the constant-overlap-add
 * condition for the Hann window, giving unbiased variance reduction.
 */
constexpr int welch_noverlap = welch_nperseg / 2;

} // namespace CFG

#endif // CONFIG_HPP
