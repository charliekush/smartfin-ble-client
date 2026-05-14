/**
 * @file filtfilt.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Zero-phase digital filtering.
 * @date 2026-05-12
 */
#ifndef FILTFILT_HPP
#define FILTFILT_HPP

#include "butterworth.hpp"

#include <vector>

namespace sf::filter {

/**
 * @brief Apply a zero-phase IIR filter to a signal.
 *
 * Filters the signal forward then backward, cancelling all phase distortion.
 *
 * @param coeffs  Filter coefficients produced by butterworth().
 * @param signal  Input signal samples.
 * @return        Filtered signal, same length as input.
 */
std::vector<double> filtfilt(const ButterworthCoeffs &coeffs,
                             const std::vector<double> &signal,
                             size_t irlen = 0);

} // namespace sf::filter

#endif // FILTFILT_HPP
