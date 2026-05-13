/**
 * @file filtfilt.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Zero-phase digital filtering via forward + backward IIR passes.
 * @date 2026-05-12
 *
 * @reference SciPy signal.filtfilt (odd-extension padding):
 *   scipy/signal/_signaltools.py — Jones E, Oliphant E, Peterson P, et al.
 *   SciPy: Open Source Scientific Tools for Python. http://www.scipy.org/
 */

#include "filter/filtfilt.hpp"
#include "math/matrix.hpp"


#include <stdexcept>

namespace sf::filter {

static ButterworthCoeffs format_coeffs(const ButterworthCoeffs& coeffs)
{
    std::vector<double> b = coeffs.b;
    std::vector<double> a = coeffs.a;
    while (!a.empty() && coeffs.a[0] == 0.0) {
        a.erase(a.begin());
    }

    if (a[0] != 1.0)
    {
        const double norm = a[0];
        for (int a_idx = 0; a_idx < a.size(); a_idx++)
            a[a_idx] /= norm;
        for (int b_idx = 0; b_idx < b.size(); b_idx++)
            b[b_idx] /= norm;
    }
    if (a.size() != b.size())
    {
        if (a.size() > b.size())
            b.resize(b.size() + a.size() - b.size());
        else
            a.resize(a.size() + b.size() - a.size());
    }
    ButterworthCoeffs new_coeffs;
    new_coeffs.a = a;
    new_coeffs.b = b;
    return new_coeffs;

}

static std::vector<double> lfilter(const ButterworthCoeffs &coeffs,
                                   const std::vector<double> &signal,
                                   std::vector<double> &delay_state)
{
    const auto &b = coeffs.b;
    const auto &a = coeffs.a;
    const size_t filter_order = b.size() - 1;
    const size_t num_samples = signal.size();

    // FIR path: after format_coeffs, a[0]=1 and a[k]=0 for k>=1
    bool is_fir = true;
    for (size_t k = 1; k < a.size(); ++k)
    {
        if (a[k] != 0.0) { is_fir = false; break; }
    }

    if (is_fir)
    {
        // Full linear convolution of b and signal: length = num_samples + filter_order
        std::vector<double> out_full(num_samples + filter_order, 0.0);
        for (size_t i = 0; i < b.size(); ++i)
            for (size_t j = 0; j < num_samples; ++j)
                out_full[i + j] += b[i] * signal[j];

        // Add initial conditions into the leading output samples
        for (size_t k = 0; k < delay_state.size(); ++k)
            out_full[k] += delay_state[k];

        // Update delay_state with the final filter_order samples (final state zf)
        delay_state.assign(out_full.begin() + num_samples, out_full.end());

        // Crop to input signal length
        out_full.resize(num_samples);
        return out_full;
    }

    // IIR path: direct form II transposed
    if (delay_state.empty())
        delay_state.assign(filter_order, 0.0);

    std::vector<double> output(num_samples);
    for (size_t n = 0; n < num_samples; ++n)
    {
        double xn = signal[n];
        double yn = b[0] * xn + delay_state[0];
        for (size_t k = 0; k < filter_order - 1; ++k)
            delay_state[k] = b[k + 1] * xn - a[k + 1] * yn + delay_state[k + 1];
        delay_state[filter_order - 1] = b[filter_order] * xn - a[filter_order] * yn;
        output[n] = yn;
    }
    return output;
}


static std::vector<double> lfilter_zi(const ButterworthCoeffs &coeffs)
{
    const auto &b = coeffs.b;
    const auto &a = coeffs.a;
    sf::math::Matrix<double> IminusA(a.size(), a.size(), 0.0);

    for (size_t row = 0; row < a.size(); ++row)
    {
        for (size_t col = 0; col < a.size(); ++col)
        {
            double companion_t = 0.0;

            if (col == 0)
            {
                companion_t = -a[row + 1];
            }
            else if (row == col - 1)
            {
                companion_t = 1.0;
            }

            IminusA(row, col) -= companion_t;
        }
    }

    std::vector<double> state_space_vec(b.begin() + 1, b.end());
    double a_scale = b[0];
    for (size_t idx = 1; idx < a.size(); idx++)
        state_space_vec[idx] -= a[idx] * a_scale;
    
    return IminusA.solveLinearSystem(state_space_vec);
    
}

std::vector<double> filtfilt(const ButterworthCoeffs &original_coeffs,
                             const std::vector<double> &signal)
{
    const ButterworthCoeffs coeffs = format_coeffs(original_coeffs);
    const size_t num_samples = signal.size();

    if (num_samples < 2)
        throw std::invalid_argument(
            "filtfilt: signal must have at least 2 samples");

    const size_t filter_order = coeffs.b.size() - 1;
    const size_t pad_len      = 3 * (filter_order + 1);

    if (num_samples <= pad_len)
        throw std::invalid_argument(
            "filtfilt: signal too short");
    return {};
}

} // namespace sf::filter
