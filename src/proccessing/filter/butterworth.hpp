/**
 * @file butterworth.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Butterworth filter design in second-order sections (SOS) form.
 * @date 2026-05-14
 */
#ifndef BUTTERWORTH_HPP
#define BUTTERWORTH_HPP

#include <array>
#include <vector>

namespace sf::filter {

/**
 * @brief Selects lowpass or highpass filter topology.
 */
enum class FilterType
{
    Lowpass,  ///< Zeros at z = -1 (Nyquist); passes low frequencies.
    Highpass, ///< Zeros at z = +1 (DC); passes high frequencies.
};

/**
 * @brief IIR filter coefficients in direct-form II transposed (b/a).
 *
 * Used internally by filtfilt and retained for oracle test vectors that
 * supply raw b/a arrays directly.
 */
struct ButterworthCoeffs
{
    std::vector<double> b; ///< Numerator polynomial coefficients.
    std::vector<double> a; ///< Denominator polynomial coefficients.
};

/**
 * @brief One second-order section (biquad) in direct-form II transposed.
 *
 * The transfer function for this section is:
 * @code
 *   H(z) = (b[0] + b[1]*z^-1 + b[2]*z^-2)
 *        / (a[0] + a[1]*z^-1 + a[2]*z^-2)
 * @endcode
 * where a[0] is always 1.
 */
struct Sos
{
    std::array<double, 3> b; ///< Numerator coefficients [b0, b1, b2].
    std::array<double, 3> a; ///< Denominator coefficients [1, a1, a2].
};

/**
 * @brief Butterworth filter represented as a cascade of biquad sections.
 *
 * An Nth-order filter produces ceil(N/2) sections.  Odd-order designs carry
 * one first-order section stored as a biquad with b[2] = a[2] = 0.
 */
using SosCoeffs = std::vector<Sos>;

/**
 * @brief Design a Butterworth filter as a cascade of second-order sections.
 *
 * Conjugate pole pairs are each mapped to one biquad, keeping coefficient
 * magnitudes well-conditioned even at very low normalized cutoff frequencies.
 * The overall passband gain is normalized to unity by scaling the first
 * section.
 *
 * @param order          Filter order (>= 1).
 * @param cutoff_hz      -3 dB cutoff frequency in Hz.
 * @param sample_rate_hz Sample rate of the signal to be filtered in Hz.
 * @param type           Lowpass or Highpass.
 * @return               SosCoeffs with ceil(order/2) biquad sections.
 */
SosCoeffs butterworth(int order, double cutoff_hz,
                      double sample_rate_hz, FilterType type);

} // namespace sf::filter

#endif // BUTTERWORTH_HPP
