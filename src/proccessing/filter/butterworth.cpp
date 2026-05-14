/**
 * @file butterworth.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Butterworth filter coefficient design in second-order sections form.
 * @date 2026-05-14
 *
 * Reference: scipy/signal/_filter_design.py
 */

#include "filter/butterworth.hpp"
#include <cmath>
#include <complex>
#include <numbers>

namespace sf::filter {

SosCoeffs butterworth(int order, double cutoff_hz, double sample_rate_hz,
                      FilterType type)
{
    using namespace std::complex_literals;

    // Normalize to [0,1] then pre-warp to analog frequency.
    const double wn     = cutoff_hz / (sample_rate_hz / 2.0);
    const double warped = 4.0 * std::tan(std::numbers::pi * wn / 2.0);

    // Analog prototype poles on a circle of radius `warped` in the left
    // half-plane.  Conjugate pairs sit at indices (k, order-1-k).
    std::vector<std::complex<double>> poles(order);
    for (int k = 0; k < order; ++k)
    {
        const double m = static_cast<double>(-order + 1 + 2 * k);
        poles[k] = -std::exp(-1.0i * std::numbers::pi * m /
                             (2.0 * static_cast<double>(order)));
        poles[k] *= warped;
    }

    // Bilinear transform: s -> z = (4 + s) / (4 - s).
    for (auto &p : poles)
        p = (4.0 + p) / (4.0 - p);

    // All zeros land at z = -1 (lowpass) or z = +1 (highpass).
    const double zero_val = (type == FilterType::Highpass) ? 1.0 : -1.0;

    const int num_sections = (order + 1) / 2; // ceil(order/2)
    SosCoeffs sos(num_sections);

    for (int i = 0; i < num_sections; ++i)
    {
        // Last section of an odd-order filter is a real first-order stage.
        const bool is_first_order = (order % 2 == 1) && (i == num_sections - 1);

        double a1, a2, b1, b2;
        if (is_first_order)
        {
            // poles[order/2] is the lone real pole (m = 0 -> purely real after
            // bilinear).
            a1 = -poles[order / 2].real();
            a2 = 0.0;
            b1 = -zero_val;
            b2 = 0.0;
        }
        else
        {
            // Conjugate pair: poles[i] and poles[order-1-i].
            const auto p = poles[i];
            a1 = -2.0 * p.real();
            a2 = std::norm(p); // |p|^2 = Re^2+Im^2
            b1 = -2.0 * zero_val;
            b2 = 1.0; // zero_val^2 = 1 for +/-1
        }

        sos[i] = {{{1.0, b1, b2}}, {{1.0, a1, a2}}};
    }

    // Evaluate total passband gain and apply its reciprocal to the first
    // section so the cascade has unity gain in the passband.
    // Reference point: z = +1 (DC) for lowpass, z = -1 (Nyquist) for highpass.
    const double ez = (type == FilterType::Highpass) ? -1.0 : 1.0;
    double total_gain = 1.0;
    for (const auto &s : sos)
    {
        const double num = s.b[0] + s.b[1] * ez + s.b[2] * ez * ez;
        const double den = s.a[0] + s.a[1] * ez + s.a[2] * ez * ez;
        if (std::abs(den) > 1e-12)
            total_gain *= num / den;
    }

    if (std::abs(total_gain) > 1e-12)
    {
        const double inv_gain = 1.0 / total_gain;
        sos[0].b[0] *= inv_gain;
        sos[0].b[1] *= inv_gain;
        sos[0].b[2] *= inv_gain;
    }

    return sos;
}

} // namespace sf::filter
