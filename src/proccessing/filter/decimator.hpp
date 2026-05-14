/**
 * @file decimator.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Streaming FIR decimator for sample-by-sample input.
 * @date 2026-05-14
 */
#ifndef DECIMATOR_HPP
#define DECIMATOR_HPP

#include <array>

namespace sf::filter {

/**
 * @brief Streaming FIR decimator with a fixed 65-tap anti-aliasing filter.
 *
 * Designed for 55 Hz input decimated by a factor of 11 to 5 Hz output.
 * Each call to push() advances an internal ring buffer by one sample and
 * produces an output every kFactor inputs.
 *
 * Not thread-safe. Call reset() between independent signal segments.
 */
class Decimator
{
public:
    static constexpr int kTaps   = 65; ///< Number of FIR filter taps.
    static constexpr int kFactor = 11; ///< Decimation factor (input rate / output rate).

    /// @brief Construct a zeroed decimator.
    Decimator();

    /**
     * @brief Push one input sample and optionally produce a decimated output.
     *
     * @param sample  Next input sample at the original rate.
     * @param out     Populated with the decimated output when this returns true.
     * @return        True every kFactor calls when an output sample is ready.
     */
    bool push(double sample, double &out);

    /**
     * @brief Reset internal buffer and counters to the initial zeroed state.
     *
     * Call between independent signal segments to prevent state bleed.
     */
    void reset();

private:
    std::array<double, kTaps> buf_{}; ///< Circular sample buffer.
    int write_idx_   = 0;             ///< Next write position in buf_.
    int input_count_ = 0;             ///< Total samples pushed since last reset.
};

} // namespace sf::filter

#endif // DECIMATOR_HPP
