/**
 * @file butterworth.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Butterworth lowpass filter design.
 * @date 2026-05-12
 */
#ifndef BUTTERWORTH_HPP
#define BUTTERWORTH_HPP

#include <vector>

namespace sf::filter {

/**
 * @brief IIR filter coefficients in direct-form II transposed (b/a).
 */
struct ButterworthCoeffs {
    
    std::vector<double> b; ///< Numerator polynomial coefficients.
    std::vector<double> a; ///< Denominator polynomial coefficients.
};

/**
 * @brief Design a Butterworth lowpass filter.
 *
 * @param order        Filter order.
 * @param cutoff_hz    -3 dB cutoff frequency in Hz.
 * @param sample_rate_hz  Sample rate of the signal to be filtered in Hz.
 * @return ButterworthCoeffs holding b and a vectors of length order+1.
 */
ButterworthCoeffs butterworth(int order, double cutoff_hz, double sample_rate_hz);

} // namespace sf::filter

#endif // BUTTERWORTH_HPP
