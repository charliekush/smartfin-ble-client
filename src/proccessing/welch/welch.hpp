/**
 * @file welch.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Welch power spectral density estimator.
 * @date 2026-05-14
 */
#ifndef WELCH_HPP
#define WELCH_HPP

#include <cstdint>
#include <vector>

namespace sf::welch {

/**
 * @brief One-sided power spectral density from a Welch estimate.
 *
 * freqs[i] and psd[i] are paired; both have length nperseg/2 + 1.
 * psd is in units of (input unit)^2 / Hz.
 */
struct WelchResult
{
    std::vector<double> freqs; ///< Frequency bins in Hz, 0 to fs/2 inclusive.
    std::vector<double> psd;   ///< One-sided PSD in (unit)^2/Hz.
    double              df;    ///< Frequency resolution in Hz (= fs / nperseg).
};

/**
 * @brief Estimate the power spectral density using the Welch method.
 *
 * Segments the signal with 50% overlap, applies a Hann window to each
 * segment, computes the periodogram via DFT, and averages across segments.
 * The result is scaled to physical one-sided PSD units.
 *
 * @param signal   Input time-series sampled at @p fs Hz.
 * @param fs       Sample rate in Hz.
 * @param nperseg  DFT length and samples per segment. Must be even and
 *                 satisfy nperseg >= 4.
 * @param noverlap Samples of overlap between adjacent segments.
 *                 Must satisfy 0 <= noverlap < nperseg.
 * @return         WelchResult with freqs and psd of length nperseg/2 + 1.
 */
WelchResult welch(const std::vector<double> &signal, double fs,
                  int nperseg = 256, int noverlap = 128);

} // namespace sf::welch

#endif // WELCH_HPP
