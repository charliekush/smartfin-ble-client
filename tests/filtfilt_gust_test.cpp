#include "filter/filtfilt.hpp"
#include "filter/butterworth.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct TestCase {
    std::string                    label;
    sf::filter::ButterworthCoeffs  coeffs;
    std::vector<double>            x;
    std::vector<double>            expected;
    size_t                         irlen = 0;
};

std::vector<double> parse_vec(std::istringstream &iss)
{
    std::vector<double> v;
    double val;
    while (iss >> val)
        v.push_back(val);
    return v;
}

std::vector<TestCase> load_cases()
{
    // Run the Python script and capture stdout
    const std::string cmd =
        "python3 " TEST_DATA_DIR "/gen_filtfilt_vectors.py 2>&1";

    FILE *pipe = popen(cmd.c_str(), "r");
    if (!pipe)
        throw std::runtime_error("popen failed: " + cmd);

    std::string output;
    std::array<char, 256> buf;
    while (fgets(buf.data(), buf.size(), pipe))
        output += buf.data();
    pclose(pipe);

    std::vector<TestCase> cases;
    TestCase cur;
    std::istringstream stream(output);
    std::string line;

    while (std::getline(stream, line))
    {
        if (line.empty()) continue;
        std::istringstream ls(line);
        std::string tag;
        ls >> tag;

        if (tag == "case")
        {
            cur = {};
            ls >> cur.label;
        }
        else if (tag == "b")     { cur.coeffs.b = parse_vec(ls); }
        else if (tag == "a")     { cur.coeffs.a = parse_vec(ls); }
        else if (tag == "x")     { cur.x        = parse_vec(ls); }
        else if (tag == "y")     { cur.expected  = parse_vec(ls); }
        else if (tag == "irlen") { ls >> cur.irlen; }
        else if (tag == "end")   { cases.push_back(cur); }
    }

    if (cases.empty())
        throw std::runtime_error(
            "No test cases parsed. Python output:\n" + output);

    return cases;
}

class FiltfiltGustTest : public testing::TestWithParam<TestCase> {};

TEST_P(FiltfiltGustTest, MatchesScipy)
{
    const auto &tc = GetParam();

    auto result = sf::filter::filtfilt(tc.coeffs, tc.x, tc.irlen);

    ASSERT_EQ(result.size(), tc.expected.size());

    double max_err = 0.0;
    std::cout << "result.size() = " << result.size() << "\n";
    for (size_t i = 0; i < result.size(); ++i)
        max_err = std::max(max_err, std::abs(result[i] - tc.expected[i]));
    std::cout << "max_err = " << max_err << "\n";
    // Normal-equations lstsq loses ~3 digits vs QR for high-order tight-cutoff
    // filters
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