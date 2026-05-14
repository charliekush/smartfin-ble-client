/**
 * @file filtfilt_gust_test.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Oracle tests comparing filtfilt output against SciPy ground truth.
 * @date 2026-05-14
 *
 * Test vectors are generated at build time by gen_filtfilt_vectors.py using
 * scipy.signal.filtfilt(method='gust') and baked into filtfilt_test_vectors.hpp.
 * Each case feeds the same coefficients and input signal to both implementations
 * and asserts element-wise agreement within a numerical tolerance.
 */

#include "filter/filtfilt.hpp"
#include "filter/butterworth.hpp"
#include "filtfilt_test_vectors.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

namespace {

/**
 * @brief One filtfilt test case loaded from the generated header.
 */
struct TestCase {
    std::string                   label;   ///< Human-readable case name.
    sf::filter::ButterworthCoeffs coeffs;  ///< Filter coefficients (b, a).
    std::vector<double>           x;       ///< Input signal.
    std::vector<double>           expected;///< SciPy ground-truth output.
    size_t                        irlen = 0; ///< Impulse-response length hint (0 = auto).
};

/**
 * @brief Convert the raw C arrays in gen::kCases to a vector of TestCase.
 *
 * @return All test cases from the generated header.
 */
std::vector<TestCase> load_cases()
{
    std::vector<TestCase> cases;
    cases.reserve(gen::kNumCases);
    for (size_t i = 0; i < gen::kNumCases; ++i) {
        const auto &r = gen::kCases[i];
        TestCase tc;
        tc.label    = r.label;
        tc.coeffs.b = {r.b, r.b + r.nb};
        tc.coeffs.a = {r.a, r.a + r.na};
        tc.x        = {r.x, r.x + r.nx};
        tc.expected = {r.y, r.y + r.ny};
        tc.irlen    = r.irlen;
        cases.push_back(std::move(tc));
    }
    return cases;
}

/**
 * @brief Parameterized fixture; one instantiation per entry in load_cases().
 */
class FiltfiltGustTest : public testing::TestWithParam<TestCase> {};

/**
 * @brief Assert that the C++ filtfilt output matches SciPy sample-for-sample.
 *
 * Tolerance is relaxed for high-order filters because the normal-equations
 * least-squares solve loses ~3 digits vs QR for tight-cutoff designs.
 */
TEST_P(FiltfiltGustTest, MatchesScipy)
{
    const auto &tc = GetParam();

    auto result = sf::filter::filtfilt(tc.coeffs, tc.x, tc.irlen);

    ASSERT_EQ(result.size(), tc.expected.size());

    double max_err = 0.0;
    for (size_t i = 0; i < result.size(); ++i)
        max_err = std::max(max_err, std::abs(result[i] - tc.expected[i]));

    /// High-order (>5 coefficients) filters use a looser tolerance.
    const double tol = (tc.coeffs.b.size() > 5) ? 1e-6 : 1e-10;
    EXPECT_LT(max_err, tol)
        << "Case '" << tc.label << "': max element-wise error = " << max_err;
}

INSTANTIATE_TEST_SUITE_P(
    SciPyGroundTruth,
    FiltfiltGustTest,
    testing::ValuesIn(load_cases()),
    [](const testing::TestParamInfo<TestCase> &info) {
        return info.param.label;
    });

} // namespace
