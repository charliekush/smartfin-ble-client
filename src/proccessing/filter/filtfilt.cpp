/**
 * @file filtfilt.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Zero-phase digital filtering via forward + backward IIR passes.
 * @date 2026-05-12
 *
 * @reference SciPy signal.filtfilt (Gustafsson initial conditions):
 *   scipy/signal/_signaltools.py — Jones E, Oliphant E, Peterson P, et al.
 *   SciPy: Open Source Scientific Tools for Python. http://www.scipy.org/
 */

#include "filter/filtfilt.hpp"
#include "math/matrix.hpp"

#include <algorithm>
#include <stdexcept>

namespace sf::filter {

/**
 * @brief Normalize filter coefficients so a[0]=1 and len(b)==len(a).
 *
 * Equal lengths are required because the direct-form II recurrence indexes
 * b[k] and a[k] in lockstep; trailing zeros are harmless.
 *
 * @param coeffs  Raw Butterworth coefficients.
 * @return        Normalized copy with a[0]=1 and len(b)==len(a).
 */
static ButterworthCoeffs format_coeffs(const ButterworthCoeffs& coeffs)
{
    std::vector<double> b = coeffs.b;
    std::vector<double> a = coeffs.a;
    // Drop leading zeros to avoid dividing by zero below.
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

/**
 * @brief Compute the initial filter state for a unit-step input (scipy lfilter_zi).
 *
 * The steady-state delay vector z satisfies z = A_companion * z + rhs,
 * so we solve (I - A_companion) * z = rhs using the companion matrix transpose.
 *
 * @param coeffs  Normalized filter coefficients (a[0] must be 1).
 * @return        Initial delay state vector of length len(a) - 1.
 */
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

    // rhs = b[1:] - a[1:] * b[0]  (steady-state driven by a unit input)
    std::vector<double> state_space_vec(b.begin() + 1, b.end());
    double a_scale = b[0];
    for (size_t idx = 1; idx < a.size(); idx++)
        state_space_vec[idx] -= a[idx] * a_scale;

    return IminusA.solveLinearSystem(state_space_vec);
}

/**
 * @brief Apply a direct-form II IIR filter to a 1-D signal.
 *
 * Supports both forward (reversed=false) and backward (reversed=true) passes
 * without physically reversing the output buffer.
 *
 * @param coeffs           Normalized filter coefficients.
 * @param signal           Input signal samples.
 * @param delasignal_state Delay state; updated in place to the final state.
 * @param reversed         If true, process samples from last to first.
 * @return                 Filtered output with the same length as @p signal.
 */
static std::vector<double> lfilter(const ButterworthCoeffs &coeffs,
                                   const std::vector<double> &signal,
                                   std::vector<double> &delasignal_state,
                                   bool reversed = false)
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
        for (size_t k = 0; k < delasignal_state.size(); ++k)
            out_full[k] += delasignal_state[k];

        // Update delasignal_state with the final filter_order samples (final
        // state zf)
        delasignal_state.assign(out_full.begin() + num_samples, out_full.end());

        // Crop to input signal length
        out_full.resize(num_samples);
        return out_full;
    }

    // IIR path: direct form II transposed.
    // The reversed flag lets us run the filter backwards without actually
    // reversing the output array — we just walk the input from the end.
    if (delasignal_state.empty())
        delasignal_state.assign(filter_order, 0.0);

    std::vector<double> output(num_samples);
    for (size_t i = 0; i < num_samples; ++i)
    {
        size_t n = reversed ? (num_samples - 1 - i) : i;
        double xn = signal[n];
        double yn = b[0] * xn + delasignal_state[0];
        for (size_t k = 0; k < filter_order - 1; ++k)
            delasignal_state[k] =
                b[k + 1] * xn - a[k + 1] * yn + delasignal_state[k + 1];
        delasignal_state[filter_order - 1] =
            b[filter_order] * xn - a[filter_order] * yn;
        output[n] = yn;
    }
    return output;
}

/**
 * @brief Apply a direct-form II IIR filter to each column of a matrix.
 *
 * Equivalent to axis=0 filtering: each column is filtered independently with
 * zero initial conditions.
 *
 * @param coeffs   Normalized filter coefficients.
 * @param input    Input matrix (rows = samples, columns = channels).
 * @param reversed If true, process each column from last row to first.
 * @return         Filtered matrix with the same dimensions as @p input.
 */
static sf::math::Matrix<double> lfilter(const ButterworthCoeffs &coeffs,
                                        const sf::math::Matrix<double> &input,
                                        bool reversed = false)
{
    const size_t nrows = input.rows();
    const size_t ncols = input.cols();
    sf::math::Matrix<double> output(nrows, ncols, 0.0);

    for (size_t col = 0; col < ncols; ++col)
    {
        std::vector<double> col_signal = input.get_col(col);
        std::vector<double> state; // zero initial conditions
        std::vector<double> col_out =
            lfilter(coeffs, col_signal, state, reversed);
        output.set_col(col, col_out);
    }
    return output;
}

std::vector<double> filtfilt(const ButterworthCoeffs &original_coeffs,
                             const std::vector<double> &signal, size_t irlen)
{
    const ButterworthCoeffs coeffs = format_coeffs(original_coeffs);
    const std::vector<double> &a = coeffs.a;
    const std::vector<double> &b = coeffs.b;
    const size_t num_samples = signal.size();

    if (num_samples < 2)
        throw std::invalid_argument(
            "filtfilt: signal must have at least 2 samples");

    const size_t filter_order = coeffs.b.size() - 1;
    const size_t pad_len = 3 * (filter_order + 1);

    if (num_samples <= pad_len)
        throw std::invalid_argument("filtfilt: signal too short");

    // For a 1st-order IIR, forward then backward exactly cancels the phase
    // shift, leaving only a gain of (b[0]/a[0])^2.  Skip the IC correction.
    if (filter_order == 1)
    {
        double scale = (b[0] / a[0]) * (b[0] / a[0]);
        std::vector<double> filtered(num_samples);
        std::transform(signal.begin(), signal.end(), filtered.begin(),
                       [scale](double val) { return val * scale; });
        return filtered;
    }
    const size_t impulse_len =
        (irlen == 0 || num_samples <= 2 * irlen) ? num_samples : irlen;

    // Build the observability matrix: column 0 is the filter's impulse
    // response, column k is that same response shifted right by k samples.
    // Together the columns span the output space reachable by the initial state.
    sf::math::Matrix<double> obs(impulse_len, filter_order, 0.0);
    std::vector<double> zi(filter_order, 0.0);
    zi[0] = 1.0;
    std::vector<double> zeros(impulse_len, 0.0);
    std::vector<double> filtered_out = lfilter(coeffs, zeros, zi);

    obs.set_col(0, filtered_out);
    for (size_t k = 1; k < filter_order; k++)
    {
        for (size_t row = k; row < impulse_len; ++row)
            obs(row, k) = obs(row - k, 0);
    }
    sf::math::Matrix<double> obs_reversed = obs.flip_rows();
    sf::math::Matrix<double> obs_rev_filtered = lfilter(coeffs, obs_reversed);
    sf::math::Matrix<double> obs_rev_filtered_reversed =
        obs_rev_filtered.flip_rows();

    // ic_mismatch encodes how a perturbation in the initial conditions changes
    // the discrepancy between y_bf and y_fb at the signal boundaries.
    const size_t nrows =
        (impulse_len == num_samples) ? impulse_len : 2 * impulse_len;
    sf::math::Matrix<double> ic_mismatch(nrows, 2 * filter_order, 0.0);

    const size_t obsr_s_row = (impulse_len == num_samples) ? 0 : impulse_len;

    for (size_t row = 0; row < impulse_len; ++row)
        for (size_t col = 0; col < filter_order; ++col)
        {
            ic_mismatch(row, col) =
                obs_rev_filtered_reversed(row, col) - obs(row, col);
            ic_mismatch(obsr_s_row + row, filter_order + col) =
                obs_reversed(row, col) - obs_rev_filtered(row, col);
        }

    // Run the four filter passes needed to measure the mismatch.
    // y_fb = forward-then-backward,  y_bf = backward-then-forward.
    std::vector<double> state;
    auto y_f = lfilter(coeffs, signal, state);
    state.clear();
    auto y_fb = lfilter(coeffs, y_f, state, true);
    state.clear();
    auto y_b = lfilter(coeffs, signal, state, true);
    state.clear();
    auto y_bf = lfilter(coeffs, y_b, state);

    // delta is the residual we want the optimal ICs to eliminate.
    std::vector<double> delta_y_bf_fb(num_samples);
    for (size_t i = 0; i < num_samples; ++i)
        delta_y_bf_fb[i] = y_bf[i] - y_fb[i];

    // When irlen < num_samples, only the boundary regions matter — take the
    // first and last impulse_len samples of the residual.
    std::vector<double> delta;
    if (impulse_len == num_samples)
    {
        delta = delta_y_bf_fb;
    }
    else
    {
        delta.resize(2 * impulse_len);
        std::copy(delta_y_bf_fb.begin(), delta_y_bf_fb.begin() + impulse_len,
                  delta.begin());
        std::copy(delta_y_bf_fb.end() - impulse_len, delta_y_bf_fb.end(),
                  delta.begin() + impulse_len);
    }

    // Solve the normal equations M^T M * ic = M^T * delta to find the initial
    // conditions that bring y_bf and y_fb into closest agreement.
    const size_t two_order = 2 * filter_order;
    sf::math::Matrix<double> MtM(two_order, two_order, 0.0);
    std::vector<double> Mtd(two_order, 0.0);
    for (size_t i = 0; i < two_order; ++i)
    {
        for (size_t j = 0; j < two_order; ++j)
            for (size_t k = 0; k < nrows; ++k)
                MtM(i, j) += ic_mismatch(k, i) * ic_mismatch(k, j);
        for (size_t k = 0; k < delta.size(); ++k)
            Mtd[i] += ic_mismatch(k, i) * delta[k];
    }
    std::vector<double> ic_opt = MtM.solveLinearSystem(Mtd);

    // W maps the IC parameter vector to per-sample output corrections.
    // Left block: forward-pass contribution (Sr); right block: backward (Obsr).
    sf::math::Matrix<double> W(nrows, two_order, 0.0);
    for (size_t row = 0; row < impulse_len; ++row)
        for (size_t col = 0; col < filter_order; ++col)
        {
            W(row, col) = obs_rev_filtered_reversed(row, col);
            W(obsr_s_row + row, filter_order + col) = obs_reversed(row, col);
        }

    // wic = ic_opt * W^T  (shape: nrows)
    std::vector<double> wic(nrows, 0.0);
    for (size_t k = 0; k < nrows; ++k)
        for (size_t j = 0; j < two_order; ++j)
            wic[k] += ic_opt[j] * W(k, j);

    // Apply the correction only at the boundary regions where the IC mismatch
    // is non-negligible; the interior is already well-conditioned.
    std::vector<double> y_opt = y_fb;
    if (impulse_len == num_samples)
    {
        for (size_t i = 0; i < num_samples; ++i)
            y_opt[i] += wic[i];
    }
    else
    {
        for (size_t i = 0; i < impulse_len; ++i)
            y_opt[i] += wic[i];
        for (size_t i = 0; i < impulse_len; ++i)
            y_opt[num_samples - impulse_len + i] += wic[impulse_len + i];
    }
    return y_opt;
}

/**
 * @brief Apply a zero-phase IIR filter to a signal via SOS cascade.
 *
 * Converts each @c Sos section to a @c ButterworthCoeffs and passes it
 * through the Gust filtfilt, so numerical precision is maintained per
 * 2nd-order section rather than across the full-order polynomial.
 *
 * @param sos     Filter in second-order sections form.
 * @param signal  Input signal samples.
 * @return        Filtered signal, same length as input.
 */
std::vector<double> filtfilt(const SosCoeffs &sos,
                             const std::vector<double> &signal)
{
    std::vector<double> y = signal;
    for (const auto &s : sos)
    {
        ButterworthCoeffs sec;
        sec.b = {s.b[0], s.b[1], s.b[2]};
        sec.a = {s.a[0], s.a[1], s.a[2]};
        y = filtfilt(sec, y);
    }
    return y;
}

} // namespace sf::filter
