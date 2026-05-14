/**
 * @file decimator.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Streaming FIR decimator for sample-by-sample input.
 * @date 2026-05-14
 */

#include "decimator.hpp"

namespace sf::filter
{
/**
 * @brief weights for kaiser FIR decimator
 *
 * @note generated with the following python script
 * import numpy as np

 * fs = 55.0
 * cutoff = 1.0
 * N = 65
 * weights = firwin(N, cutoff, fs=fs, window=('kaiser', 6))
 * print(', '.join(f'{w:.8f}f' for w in weights))
 */
static const float kDecimWeights[65] = {
    -0.00008306f, -0.00011099f, -0.00012311f, -0.00010752f, -0.00004981f,
    0.00006680f,  0.00026107f,  0.00055324f,  0.00096436f,  0.00151550f,
    0.00222677f,  0.00311633f,  0.00419938f,  0.00548705f,  0.00698547f,
    0.00869492f,  0.01060907f,  0.01271460f,  0.01499087f,  0.01741007f,
    0.01993754f,  0.02253242f,  0.02514864f,  0.02773610f,  0.03024215f,
    0.03261317f,  0.03479633f,  0.03674133f,  0.03840211f,  0.03973850f,
    0.04071762f,  0.04131511f,  0.04151595f,  0.04131511f,  0.04071762f,
    0.03973850f,  0.03840211f,  0.03674133f,  0.03479633f,  0.03261317f,
    0.03024215f,  0.02773610f,  0.02514864f,  0.02253242f,  0.01993754f,
    0.01741007f,  0.01499087f,  0.01271460f,  0.01060907f,  0.00869492f,
    0.00698547f,  0.00548705f,  0.00419938f,  0.00311633f,  0.00222677f,
    0.00151550f,  0.00096436f,  0.00055324f,  0.00026107f,  0.00006680f,
    -0.00004981f, -0.00010752f, -0.00012311f, -0.00011099f, -0.00008306f};
}

sf::filter::Decimator::Decimator()
{
    buf_.fill(0.0);
}

bool sf::filter::Decimator::push(double sample, double &out)
{
    buf_[write_idx_] = sample;
    write_idx_ = (write_idx_ + 1) % kTaps;
    ++input_count_;

    if (input_count_ % kFactor != 0)
        return false;

    double acc = 0.0;
    int sample_idx = write_idx_ - 1;
    if (sample_idx < 0)
        sample_idx = kTaps - 1;

    for (int tap = 0; tap < kTaps; ++tap)
    {
        acc += static_cast<double>(kDecimWeights[tap]) * buf_[sample_idx];
        --sample_idx;
        if (sample_idx < 0)
            sample_idx = kTaps - 1;
    }

    out = acc;
    return true;
}

void sf::filter::Decimator::reset()
{
    buf_.fill(0.0);
    write_idx_ = 0;
    input_count_ = 0;
}
