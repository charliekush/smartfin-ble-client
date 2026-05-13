/**
 * @file butterworth.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Butterworth filter coefficient design.
 * @date 2026-05-12
 *
 * Reference: scipy/signal/_filter_design.py
 */

#include "filter/butterworth.hpp"
#include <cmath>
#include <numbers>
#include <complex>

/**
 * @brief Compute monic polynomial coefficients from roots.
 * @param roots  Complex roots.
 * @return       Coefficients [1, c1, ..., cN] in descending order.
 */
static std::vector<std::complex<double>> poly(
    const std::vector<std::complex<double>> &roots)
{
    std::vector<std::complex<double>> coeffs = {1.0};
    for (auto& root : roots) {
        coeffs.push_back(0.0);
        for (int i = coeffs.size()-1; i > 0; --i)
            coeffs[i] -= root * coeffs[i-1];
    }
    return coeffs;
}

sf::filter::ButterworthCoeffs sf::filter::butterworth(int order,
                                                       double cutoff_hz,
                                                       double sample_rate_hz,
                                                       FilterType type)
{
    using namespace std::complex_literals;

    cutoff_hz = cutoff_hz / (sample_rate_hz / 2.0);
    double warped = 4.0 * std::tan(std::numbers::pi * cutoff_hz / 2.0);

    // Analog prototype poles, left half-plane.
    std::vector<std::complex<double>> poles(order);
    for (int k = 0; k < order; ++k)
    {
        double m = -order + 1 + 2 * k;
        poles[k] = -std::exp(-1.0i * std::numbers::pi * m / (2.0 * order));
        poles[k] *= warped;
    }

    // Lowpass zeros at z=-1 (kill Nyquist), highpass at z=+1 (kill DC).
    const std::complex<double> zero_val =
        (type == FilterType::Highpass) ? +1.0 : -1.0;
    std::vector<std::complex<double>> zeros_z(order, zero_val);

    // Bilinear transform: s -> z = (4+s)/(4-s).
    double transfer_func_gain = std::pow(warped, order);
    std::complex<double> pole_prod = 1.0;
    for (auto &pole : poles)
    {
        pole_prod *= (4.0 - pole);
        pole = (4.0 + pole) / (4.0 - pole);
    }
    transfer_func_gain *= std::real(1.0 / pole_prod);

    // Normalize gain: evaluate H(z) at the passband edge.
    const std::complex<double> eval_z =
        (type == FilterType::Highpass) ? -1.0 : +1.0;

    std::complex<double> num_at_eval = 1.0;
    for (const auto &z : zeros_z)
        num_at_eval *= (eval_z - z);

    std::complex<double> den_at_eval = 1.0;
    for (const auto &pole : poles)
        den_at_eval *= (eval_z - pole);

    const double passband_gain =
        std::abs(transfer_func_gain * num_at_eval / den_at_eval);
    if (passband_gain > 1e-12)
        transfer_func_gain /= passband_gain;

    auto b_complex = poly(zeros_z);
    auto a_complex = poly(poles);

    ButterworthCoeffs coeffs;
    coeffs.b.resize(b_complex.size());
    coeffs.a.resize(a_complex.size());

    for (size_t i = 0; i < b_complex.size(); ++i)
        coeffs.b[i] = transfer_func_gain * std::real(b_complex[i]);
    for (size_t i = 0; i < a_complex.size(); ++i)
        coeffs.a[i] = std::real(a_complex[i]);

    return coeffs;
}
