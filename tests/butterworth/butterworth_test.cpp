/**
 * @file butterworth_test.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Property-based tests for the Butterworth filter design.
 * @date 2026-05-14
 *
 * Verifies analytically defined guarantees of a Butterworth filter: −3 dB at
 * cutoff, unity passband gain, correct zero placement, monotonic magnitude
 * response, and coefficient structure.  No reference implementation is needed
 * because these properties follow directly from the filter specification.
 */

#include "filter/butterworth.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <complex>
#include <numbers>

namespace
{

/**
 * @brief Evaluate the magnitude response |H(e^{jω})| at a digital frequency.
 *
 * Computes the DTFT of b and a at the given frequency and returns the
 * magnitude of their ratio.
 *
 * @param c     Filter coefficients (b numerator, a denominator).
 * @param omega Digital frequency in radians/sample ∈ [0, π].
 * @return      |H(e^{jω})|
 */
double mag_response(const sf::filter::ButterworthCoeffs &c, double omega)
{
    std::complex<double> num = 0.0, den = 0.0;
    for (size_t k = 0; k < c.b.size(); ++k)
        num += c.b[k] * std::exp(std::complex<double>(0.0, -omega * double(k)));
    for (size_t k = 0; k < c.a.size(); ++k)
        den += c.a[k] * std::exp(std::complex<double>(0.0, -omega * double(k)));
    return std::abs(num / den);
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

// Lowpass

/**
 * @brief Lowpass magnitude is 1/√2 (−3 dB) at the cutoff frequency.
 *
 * Bilinear pre-warping maps the analog −3 dB point exactly to the specified
 * digital cutoff, so this should hold to near floating-point precision for
 * all filter orders.
 */
TEST(ButterworthLowpass, MinusThreeDbAtCutoff)
{
    for (int order : {1, 2, 4, 6})
    {
        auto c =
            sf::filter::butterworth(order, 10.0, 100.0, FilterType::Lowpass);
        EXPECT_NEAR(mag_response(c, omega(10.0, 100.0)), 1.0 / std::sqrt(2.0),
                    1e-6)
            << "order=" << order;
    }
}

/// @brief Lowpass gain is unity at DC (ω = 0).
TEST(ButterworthLowpass, UnityGainAtDC)
{
    auto c = sf::filter::butterworth(4, 10.0, 100.0, FilterType::Lowpass);
    EXPECT_NEAR(mag_response(c, 0.0), 1.0, 1e-9);
}

/// @brief Lowpass zeros at z = −1 drive the response to near zero at Nyquist.
TEST(ButterworthLowpass, NearZeroAtNyquist)
{
    auto c = sf::filter::butterworth(4, 10.0, 100.0, FilterType::Lowpass);
    EXPECT_LT(mag_response(c, std::numbers::pi), 1e-6);
}

/// @brief Lowpass magnitude decreases monotonically past the cutoff (maximally
/// flat).
TEST(ButterworthLowpass, MonotonicRolloff)
{
    auto c = sf::filter::butterworth(4, 10.0, 100.0, FilterType::Lowpass);
    double prev = mag_response(c, omega(10.0, 100.0));
    for (double f : {15.0, 20.0, 30.0, 40.0})
    {
        double cur = mag_response(c, omega(f, 100.0));
        EXPECT_LT(cur, prev) << "not monotone at f=" << f << " Hz";
        prev = cur;
    }
}

/// @brief Higher filter order gives greater stopband attenuation at the same
/// cutoff.
TEST(ButterworthLowpass, HigherOrderAttenuatesMoreInStopband)
{
    /// At 2× the cutoff, order 4 should reject more than order 2.
    const double fs = 100.0, fc = 10.0, f_stop = 20.0;
    auto c2 = sf::filter::butterworth(2, fc, fs, FilterType::Lowpass);
    auto c4 = sf::filter::butterworth(4, fc, fs, FilterType::Lowpass);
    EXPECT_LT(mag_response(c4, omega(f_stop, fs)),
              mag_response(c2, omega(f_stop, fs)));
}

// Highpass

/**
 * @brief Highpass magnitude is 1/√2 (−3 dB) at the cutoff frequency.
 */
TEST(ButterworthHighpass, MinusThreeDbAtCutoff)
{
    for (int order : {1, 2, 4, 6})
    {
        auto c =
            sf::filter::butterworth(order, 10.0, 100.0, FilterType::Highpass);
        EXPECT_NEAR(mag_response(c, omega(10.0, 100.0)), 1.0 / std::sqrt(2.0),
                    1e-6)
            << "order=" << order;
    }
}

/// @brief Highpass gain is unity at Nyquist (ω = π).
TEST(ButterworthHighpass, UnityGainAtNyquist)
{
    auto c = sf::filter::butterworth(4, 10.0, 100.0, FilterType::Highpass);
    EXPECT_NEAR(mag_response(c, std::numbers::pi), 1.0, 1e-9);
}

/// @brief Highpass zeros at z = +1 drive the response to near zero at DC.
TEST(ButterworthHighpass, NearZeroAtDC)
{
    auto c = sf::filter::butterworth(4, 10.0, 100.0, FilterType::Highpass);
    EXPECT_LT(mag_response(c, 0.0), 1e-6);
}

/// @brief Highpass magnitude increases monotonically above the cutoff.
TEST(ButterworthHighpass, MonotonicPassband)
{
    auto c = sf::filter::butterworth(4, 10.0, 100.0, FilterType::Highpass);
    double prev = mag_response(c, omega(10.0, 100.0));
    for (double f : {15.0, 20.0, 30.0, 40.0})
    {
        double cur = mag_response(c, omega(f, 100.0));
        EXPECT_GT(cur, prev) << "not monotone increasing at f=" << f << " Hz";
        prev = cur;
    }
}

// Coefficient structure

/// @brief b and a vectors each have length order + 1.
TEST(ButterworthCoeffs, LengthIsOrderPlusOne)
{
    for (int order : {1, 2, 4, 6})
    {
        auto c =
            sf::filter::butterworth(order, 10.0, 100.0, FilterType::Lowpass);
        EXPECT_EQ(c.b.size(), size_t(order + 1)) << "order=" << order;
        EXPECT_EQ(c.a.size(), size_t(order + 1)) << "order=" << order;
    }
}

/// @brief a[0] is normalized to 1.0 for both filter types.
TEST(ButterworthCoeffs, LeadingDenominatorIsOne)
{
    for (auto type : {FilterType::Lowpass, FilterType::Highpass})
    {
        auto c = sf::filter::butterworth(4, 10.0, 100.0, type);
        EXPECT_NEAR(c.a[0], 1.0, 1e-12);
    }
}

} // namespace
