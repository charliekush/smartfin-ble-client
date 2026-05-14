/**
 * @file filtfilt_properties_test.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Property-based tests for zero-phase IIR filtering via filtfilt.
 * @date 2026-05-14
 *
 * Verifies mathematical guarantees that hold for any correct filtfilt
 * implementation, independent of the specific algorithm or reference
 * implementation: LTI linearity, DC passthrough, zero-phase response,
 * squared-magnitude gain, and stopband attenuation.
 */

#include "filter/filtfilt.hpp"
#include "filter/butterworth.hpp"

#include <gtest/gtest.h>

#include <complex>
#include <cmath>
#include <numbers>
#include <vector>

namespace {

/**
 * @brief Evaluate the magnitude response |H(e^{jω})| for an SOS cascade.
 *
 * @param sos   Filter in second-order sections form.
 * @param omega Digital frequency in radians/sample ∈ [0, π].
 * @return      |H(e^{jω})|
 */
double sos_mag_response(const sf::filter::SosCoeffs &sos, double omega)
{
    std::complex<double> h = 1.0;
    const std::complex<double> ejw =
        std::exp(std::complex<double>(0.0, -omega));
    const std::complex<double> ej2w = ejw * ejw;
    for (const auto &s : sos)
    {
        const std::complex<double> num =
            s.b[0] + s.b[1] * ejw + s.b[2] * ej2w;
        const std::complex<double> den =
            s.a[0] + s.a[1] * ejw + s.a[2] * ej2w;
        h *= num / den;
    }
    return std::abs(h);
}

/**
 * @brief Convert a frequency in Hz to digital frequency in radians/sample.
 *
 * @param freq_hz Frequency in Hz.
 * @param fs      Sample rate in Hz.
 * @return        ω = π · freq_hz / (fs / 2)
 */
double omega(double freq_hz, double fs)
{
    return std::numbers::pi * freq_hz / (fs / 2.0);
}

using sf::filter::FilterType;

// Basic contract

/// @brief Output vector length equals input vector length for any signal size.
TEST(FiltfiltProperties, OutputLengthMatchesInput)
{
    auto c = sf::filter::butterworth(4, 10.0, 100.0, FilterType::Lowpass);
    for (size_t n : {50u, 100u, 201u})
        EXPECT_EQ(sf::filter::filtfilt(c, std::vector<double>(n, 1.0)).size(), n);
}

// LTI properties

/**
 * @brief filtfilt is linear: filtfilt(α·x1 + β·x2) == α·filtfilt(x1) + β·filtfilt(x2).
 *
 * Uses two sinusoids at different passband frequencies to ensure both
 * components are processed, not simply zeroed by the filter.
 */
TEST(FiltfiltProperties, Linearity)
{
    const double fs = 200.0;
    auto c = sf::filter::butterworth(4, 20.0, fs, FilterType::Lowpass);
    const size_t n = 300;

    std::vector<double> x1(n), x2(n), combo(n);
    const double alpha = 3.0, beta = -2.0;
    for (size_t i = 0; i < n; ++i)
    {
        x1[i]    = std::sin(2.0 * std::numbers::pi * 3.0 * double(i) / fs);
        x2[i]    = std::cos(2.0 * std::numbers::pi * 5.0 * double(i) / fs);
        combo[i] = alpha * x1[i] + beta * x2[i];
    }

    auto y_combo = sf::filter::filtfilt(c, combo);
    auto y1      = sf::filter::filtfilt(c, x1);
    auto y2      = sf::filter::filtfilt(c, x2);

    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(y_combo[i], alpha * y1[i] + beta * y2[i], 1e-10)
            << "i=" << i;
}

/// @brief Scaling the input by a constant scales the output by the same constant.
TEST(FiltfiltProperties, ScaleInvariance)
{
    auto c = sf::filter::butterworth(4, 10.0, 100.0, FilterType::Lowpass);
    const size_t n = 200;
    const double k = 7.5;

    std::vector<double> x(n), kx(n);
    for (size_t i = 0; i < n; ++i)
    {
        x[i]  = std::sin(2.0 * std::numbers::pi * 2.0 * double(i) / 100.0);
        kx[i] = k * x[i];
    }

    auto y  = sf::filter::filtfilt(c, x);
    auto ky = sf::filter::filtfilt(c, kx);

    for (size_t i = 0; i < n; ++i)
        EXPECT_NEAR(ky[i], k * y[i], 1e-10) << "i=" << i;
}

// filtfilt-specific

/**
 * @brief A constant (DC) signal is preserved unchanged by a lowpass filter.
 *
 * The lowpass gain at DC is |H(0)| = 1, so filtfilt gives gain 1² = 1.
 * Edge samples are skipped to avoid boundary transients.
 */
TEST(FiltfiltProperties, DCPassthroughLowpass)
{
    auto c = sf::filter::butterworth(4, 10.0, 100.0, FilterType::Lowpass);
    const size_t n = 300;
    const double dc = 3.7;
    auto y = sf::filter::filtfilt(c, std::vector<double>(n, dc));
    // Tolerance is 1e-5: two cascaded Gust passes (one per biquad section)
    // each contribute ~1e-6 residual, compounding to ~5e-6 near the guard band.
    for (size_t i = 50; i < n - 50; ++i)
        EXPECT_NEAR(y[i], dc, 1e-5) << "i=" << i;
}

/**
 * @brief The output is in phase with the input for a passband sinusoid.
 *
 * Measured via normalized cross-correlation: (x·y)² / (|x|²·|y|²) = 1
 * if and only if y is a non-negative scalar multiple of x (zero phase shift).
 * The middle 300 samples of a 500-sample signal are used to avoid edge effects.
 */
TEST(FiltfiltProperties, ZeroPhase)
{
    const double fs = 200.0, fc = 20.0, f_sig = 5.0;
    auto c = sf::filter::butterworth(4, fc, fs, FilterType::Lowpass);
    const size_t n = 500;

    std::vector<double> x(n);
    for (size_t i = 0; i < n; ++i)
        x[i] = std::sin(2.0 * std::numbers::pi * f_sig * double(i) / fs);

    auto y = sf::filter::filtfilt(c, x);

    double xx = 0.0, xy = 0.0, yy = 0.0;
    for (size_t i = 100; i < 400; ++i)
    {
        xx += x[i] * x[i];
        xy += x[i] * y[i];
        yy += y[i] * y[i];
    }
    EXPECT_NEAR((xy * xy) / (xx * yy), 1.0, 1e-6);
}

/**
 * @brief The effective gain at a passband frequency is |H(ω)|², not |H(ω)|.
 *
 * A single causal pass gives gain |H(ω)|; the forward + backward passes of
 * filtfilt compound to |H(ω)|².  Gain is measured as the cross-correlation
 * ratio (x·y) / |x|² in the steady-state region.
 */
TEST(FiltfiltProperties, SquaredMagnitudeResponse)
{
    const double fs = 200.0, fc = 20.0, f_sig = 5.0;
    auto c = sf::filter::butterworth(4, fc, fs, FilterType::Lowpass);
    const size_t n = 500;

    std::vector<double> x(n);
    for (size_t i = 0; i < n; ++i)
        x[i] = std::sin(2.0 * std::numbers::pi * f_sig * double(i) / fs);

    auto y = sf::filter::filtfilt(c, x);

    double xx = 0.0, xy = 0.0;
    for (size_t i = 100; i < 400; ++i)
    {
        xx += x[i] * x[i];
        xy += x[i] * y[i];
    }
    const double measured = xy / xx;
    const double expected =
        std::pow(sos_mag_response(c, omega(f_sig, fs)), 2.0);
    EXPECT_NEAR(measured, expected, 1e-4);
}

/**
 * @brief A sinusoid well into the stopband is attenuated to near zero.
 *
 * RMS of the middle 300 output samples is used to avoid edge transients and
 * to measure steady-state attenuation rather than peak values.
 */
TEST(FiltfiltProperties, StopbandAttenuation)
{
    const double fs = 200.0, fc = 10.0, f_stop = 50.0;
    auto c = sf::filter::butterworth(4, fc, fs, FilterType::Lowpass);
    const size_t n = 500;

    std::vector<double> x(n);
    for (size_t i = 0; i < n; ++i)
        x[i] = std::sin(2.0 * std::numbers::pi * f_stop * double(i) / fs);

    auto y = sf::filter::filtfilt(c, x);

    double yy = 0.0;
    for (size_t i = 100; i < 400; ++i)
        yy += y[i] * y[i];
    EXPECT_LT(std::sqrt(yy / 300.0), 1e-3);
}

} // namespace
