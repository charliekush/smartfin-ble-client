/**
 * @file config.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Project-wide scalar type and default sensor calibration constants.
 * @date 2026-05-07
 */

#ifndef CONFIG_HPP
#define CONFIG_HPP

namespace CFG
{
/// @brief Floating point precision used throughout the processing pipeline.
typedef double Scalar;

/**
 * @brief  Butterworth lowpass -3 dB corner frequency applied to oriented
 * acceleration.
 */
constexpr Scalar cutoff_fc_hz = 3.0;

/// @brief Bandpass lower edge (highpass corner) in Hz.
constexpr Scalar bandpass_lo_hz = 0.05;

/// @brief Bandpass upper edge (lowpass corner) in Hz.
constexpr Scalar bandpass_hi_hz = 0.5;
} // namespace CFG

#endif // CONFIG_HPP
