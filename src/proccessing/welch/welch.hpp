/**
 * @file welch.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Welch power spectral density estimator.
 * @date 2026-05-14
 */
#ifndef WELCH_HPP
#define WELCH_HPP

#include <complex>
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
 * @brief In-place Cooley-Tukey radix-2 DIT FFT.
 *
 * Permutes x into bit-reversed order then applies log2(N) stages of
 * butterfly operations using the precomputed kTwiddle table.
 *
 * @param x  Complex signal of length welch_nperseg. Modified in place.
 */
void fft(std::vector<std::complex<double>> &x);

/**
 * @brief Compute the one-sided squared-magnitude DFT of a real signal.
 *
 * Packs the real input into a complex vector, runs the in-place FFT, then
 * writes |X[k]|^2 for k = 0 .. CFG::welch_nperseg/2 into mag_sq. The DC
 * bin (k=0) and Nyquist bin (k=N/2) are unique; all other bins appear twice
 * in the two-sided spectrum, but only the one-sided values are written here.
 *
 * @param signal  Real input of length CFG::welch_nperseg.
 * @param mag_sq  Output of length CFG::welch_nperseg/2 + 1. Overwritten.
 */
void real_dft_mag_sq(const std::vector<double> &signal,
                     std::vector<double> &mag_sq);

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
