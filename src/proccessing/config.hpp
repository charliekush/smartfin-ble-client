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
 * At the 5 Hz decimated rate this gives a frequency resolution of
 * 5 / 256 = 0.0195 Hz, which resolves wave periods up to 51 seconds.
 * Must be a positive even integer and satisfy nperseg >= 4.
 */
constexpr int welch_nperseg = 256;

/**
 * @brief Overlap between adjacent Welch segments in samples.
 *
 * Set to nperseg / 2 (50% overlap). Satisfies the constant-overlap-add
 * condition for the Hann window, giving unbiased variance reduction.
 */
constexpr int welch_noverlap = 128;

} // namespace CFG

#endif // CONFIG_HPP
