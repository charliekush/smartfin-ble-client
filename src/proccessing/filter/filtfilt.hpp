/**
 * @file filtfilt.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Zero-phase digital filtering.
 * @date 2026-05-14
 */
#ifndef FILTFILT_HPP
#define FILTFILT_HPP

#include "butterworth.hpp"

#include <vector>

namespace sf::filter {

/**
 * @brief Apply a zero-phase IIR filter to a signal (flat-polynomial form).
 *
 * Retained for oracle tests that supply raw b/a arrays directly. Filters the
 * signal forward then backward, cancelling all phase distortion.
 *
 * @param coeffs  Filter coefficients in b/a polynomial form.
 * @param signal  Input signal samples.
 * @param irlen   Impulse-response length hint (0 = use full signal length).
 * @return        Filtered signal, same length as input.
 */
std::vector<double> filtfilt(const ButterworthCoeffs &coeffs,
                             const std::vector<double> &signal,
                             size_t irlen = 0);

/**
 * @brief Apply a zero-phase IIR filter to a signal (SOS cascade form).
 *
 * Applies each second-order section in sequence using the Gust
 * forward-backward algorithm per section, keeping coefficient magnitudes
 * well-conditioned even at very low normalized cutoff frequencies.
 *
 * @param sos     Filter in second-order sections form from butterworth().
 * @param signal  Input signal samples.
 * @return        Filtered signal, same length as input.
 */
std::vector<double> filtfilt(const SosCoeffs &sos,
                             const std::vector<double> &signal);

} // namespace sf::filter

#endif // FILTFILT_HPP
