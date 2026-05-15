/**
 * @file fft_test.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Unit tests for the Cooley-Tukey FFT used in the Welch estimator.
 * @date 2026-05-15
 */
#include "config.hpp"

#include <complex>
#include <cmath>
#include <numbers>
#include <vector>

#include <gtest/gtest.h>

namespace sf::welch
{
void fft(std::vector<std::complex<double>> &x);
} // namespace sf::welch

using sf::welch::fft;

static constexpr int N = CFG::welch_nperseg;

/**
 * @brief Build a length-N complex vector from a real initializer.
 *
 * @param real_vals  Real parts; imaginary parts are zero.
 * @return           Complex vector of length N.
 */
static std::vector<std::complex<double>>
make_cx(const std::vector<double> &real_vals)
{
    std::vector<std::complex<double>> x(N, 0.0);
    for (int i = 0; i < static_cast<int>(real_vals.size()) && i < N; ++i)
        x[i] = real_vals[i];
    return x;
}

/**
 * @brief All-zero input produces an all-zero output.
 */
TEST(FFT, ZeroInput)
{
    auto x = make_cx(std::vector<double>(N, 0.0));
    fft(x);
    for (int k = 0; k < N; ++k)
        EXPECT_DOUBLE_EQ(x[k].real(), 0.0) << "real at bin " << k;
    for (int k = 0; k < N; ++k)
        EXPECT_DOUBLE_EQ(x[k].imag(), 0.0) << "imag at bin " << k;
}

/**
 * @brief Constant input of amplitude A produces X[0] = N*A; all other bins
 * are zero.
 */
TEST(FFT, DCInput)
{
    const double A = 3.0;
    auto x = make_cx(std::vector<double>(N, A));
    fft(x);
    EXPECT_NEAR(x[0].real(), static_cast<double>(N) * A, 1e-9);
    EXPECT_NEAR(x[0].imag(), 0.0, 1e-9);
    for (int k = 1; k < N; ++k)
    {
        EXPECT_NEAR(x[k].real(), 0.0, 1e-9) << "real at bin " << k;
        EXPECT_NEAR(x[k].imag(), 0.0, 1e-9) << "imag at bin " << k;
    }
}

/**
 * @brief Unit impulse at n=0 produces a flat magnitude spectrum: |X[k]| = 1.
 */
TEST(FFT, UnitImpulse)
{
    std::vector<double> r(N, 0.0);
    r[0] = 1.0;
    auto x = make_cx(r);
    fft(x);
    for (int k = 0; k < N; ++k)
        EXPECT_NEAR(std::abs(x[k]), 1.0, 1e-10) << "at bin " << k;
}

/**
 * @brief Cosine at integer bin k0 places |X[k0]| = |X[N-k0]| = N/2; all
 * other bins are zero.
 */
TEST(FFT, CosineAtIntegerBin)
{
    const int k0 = 4;
    std::vector<double> r(N);
    for (int n = 0; n < N; ++n)
        r[n] = std::cos(2.0 * std::numbers::pi * k0 * n / N);
    auto x = make_cx(r);
    fft(x);

    const double expected = static_cast<double>(N) / 2.0;
    EXPECT_NEAR(std::abs(x[k0]), expected, 1e-9);
    EXPECT_NEAR(std::abs(x[N - k0]), expected, 1e-9);
    for (int k = 0; k < N; ++k)
    {
        if (k == k0 || k == N - k0)
            continue;
        EXPECT_NEAR(std::abs(x[k]), 0.0, 1e-9) << "at bin " << k;
    }
}

/**
 * @brief Sine at integer bin k0 places |X[k0]| = |X[N-k0]| = N/2 with a
 * pi/2 phase shift relative to the cosine case.
 */
TEST(FFT, SineAtIntegerBin)
{
    const int k0 = 5;
    std::vector<double> r(N);
    for (int n = 0; n < N; ++n)
        r[n] = std::sin(2.0 * std::numbers::pi * k0 * n / N);
    auto x = make_cx(r);
    fft(x);

    const double expected = static_cast<double>(N) / 2.0;
    EXPECT_NEAR(std::abs(x[k0]), expected, 1e-9);
    EXPECT_NEAR(std::abs(x[N - k0]), expected, 1e-9);
    for (int k = 0; k < N; ++k)
    {
        if (k == k0 || k == N - k0)
            continue;
        EXPECT_NEAR(std::abs(x[k]), 0.0, 1e-9) << "at bin " << k;
    }
}

/**
 * @brief Parseval's theorem: sum_n |x[n]|^2 = (1/N) * sum_k |X[k]|^2.
 */
TEST(FFT, Parseval)
{
    const int k0 = 7;
    std::vector<double> r(N);
    double time_power = 0.0;
    for (int n = 0; n < N; ++n)
    {
        r[n] = std::cos(2.0 * std::numbers::pi * k0 * n / N);
        time_power += r[n] * r[n];
    }
    auto x = make_cx(r);
    fft(x);

    double freq_power = 0.0;
    for (int k = 0; k < N; ++k)
        freq_power += std::norm(x[k]);

    EXPECT_NEAR(freq_power / N, time_power, 1e-9);
}

/**
 * @brief FFT is linear: fft(a*u + b*v) = a*fft(u) + b*fft(v).
 */
TEST(FFT, Linearity)
{
    const double a = 2.0;
    const double b = -0.5;
    const int k1 = 3;
    const int k2 = 11;

    std::vector<double> u(N), v(N);
    for (int n = 0; n < N; ++n)
    {
        u[n] = std::cos(2.0 * std::numbers::pi * k1 * n / N);
        v[n] = std::sin(2.0 * std::numbers::pi * k2 * n / N);
    }

    // Compute fft(a*u + b*v) directly.
    std::vector<double> combo(N);
    for (int n = 0; n < N; ++n)
        combo[n] = a * u[n] + b * v[n];
    auto xu = make_cx(u);
    auto xv = make_cx(v);
    auto xc = make_cx(combo);
    fft(xu);
    fft(xv);
    fft(xc);

    for (int k = 0; k < N; ++k)
    {
        const std::complex<double> expected = a * xu[k] + b * xv[k];
        EXPECT_NEAR(xc[k].real(), expected.real(), 1e-9) << "real at bin " << k;
        EXPECT_NEAR(xc[k].imag(), expected.imag(), 1e-9) << "imag at bin " << k;
    }
}

/**
 * @brief Complex exponential e^{j*2*pi*k0*n/N} produces a single nonzero bin.
 *
 * This exercises the imaginary input path. X[k0] = N; all other bins are zero.
 */
TEST(FFT, ComplexExponential)
{
    const int k0 = 6;
    std::vector<std::complex<double>> x(N);
    for (int n = 0; n < N; ++n)
    {
        const double theta = 2.0 * std::numbers::pi * k0 * n / N;
        x[n] = {std::cos(theta), std::sin(theta)};
    }
    fft(x);

    EXPECT_NEAR(std::abs(x[k0]), static_cast<double>(N), 1e-9);
    for (int k = 0; k < N; ++k)
    {
        if (k == k0)
            continue;
        EXPECT_NEAR(std::abs(x[k]), 0.0, 1e-9) << "at bin " << k;
    }
}
