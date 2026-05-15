/**
 * @file fft.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Cooley-Tukey radix-2 decimation-in-time FFT for the Welch estimator.
 * @date 2026-05-14
 */
#include "config.hpp"
#include <array>
#include <cmath>
#include <complex>
#include <numbers>
#include <utility>
#include <vector>

namespace sf::welch
{

static const auto kTwiddle = []()
{
    std::array<std::pair<double, double>, CFG::welch_nperseg / 2> t;
    for (int k = 0; k < CFG::welch_nperseg / 2; ++k)
    {
        const double theta = 2.0 * std::numbers::pi * k / CFG::welch_nperseg;
        t[k] = {std::cos(theta), -std::sin(theta)};
    }
    return t;
}();

static const int log2n = static_cast<int>(std::log2(CFG::welch_nperseg));

/**
 * @brief Reverse the lower log2n bits of i.
 *
 * @param i  Index to bit-reverse.
 * @return   Bit-reversed index.
 */
static int bit_reverse(int i)
{
    int r = 0;
    for (int b = 0; b < log2n; ++b)
    {
        r = (r << 1) | (i & 1);
        i >>= 1;
    }
    return r;
}

/**
 * @brief In-place Cooley-Tukey radix-2 DIT FFT.
 *
 * Permutes x into bit-reversed order then applies log2(N) stages of
 * butterfly operations using the precomputed kTwiddle table.
 *
 * @param x  Complex signal of length welch_nperseg. Modified in place.
 */
void fft(std::vector<std::complex<double>> &x)
{
    for (int i = 0; i < CFG::welch_nperseg; ++i)
    {
        int j = bit_reverse(i);
        if (j > i)
            std::swap(x[i], x[j]);
    }

    for (int stage = 1; stage <= log2n; ++stage)
    {
        const int half = 1 << (stage - 1);
        for (int j = 0; j < CFG::welch_nperseg; j += 2 * half)
        {
            for (int k = 0; k < half; ++k)
            {
                const auto &[wr, wi] =
                    kTwiddle[k * (CFG::welch_nperseg / 2 / half)];
                const std::complex<double> w(wr, wi);
                const std::complex<double> u = x[j + k];
                const std::complex<double> v = w * x[j + k + half];
                x[j + k]        = u + v;
                x[j + k + half] = u - v;
            }
        }
    }
}

/**
 * @brief Compute the one-sided squared-magnitude DFT of a real signal.
 *
 * Packs the real input into a complex vector, runs the in-place FFT, then
 * writes |X[k]|^2 for k = 0 .. welch_nperseg/2 into mag_sq.  The DC bin
 * (k=0) and Nyquist bin (k=N/2) are unique; all other bins appear twice in
 * the two-sided spectrum, but only the one-sided values are written here.
 *
 * @param signal  Real input of length welch_nperseg.
 * @param mag_sq  Output of length welch_nperseg/2 + 1.  Overwritten.
 */
void real_dft_mag_sq(const std::vector<double> &signal,
                     std::vector<double> &mag_sq)
{
    std::vector<std::complex<double>> x(CFG::welch_nperseg);
    for (int i = 0; i < CFG::welch_nperseg; ++i)
        x[i] = signal[i];

    fft(x);

    const int bins = CFG::welch_nperseg / 2 + 1;
    mag_sq.resize(bins);
    for (int k = 0; k < bins; ++k)
        mag_sq[k] = std::norm(x[k]);
}

} // namespace sf::welch
