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
static const double kDecimWeights[65] = {
    -0.00008306, -0.00011099, -0.00012311, -0.00010752, -0.00004981,
    0.00006680,  0.00026107,  0.00055324,  0.00096436,  0.00151550,
    0.00222677,  0.00311633,  0.00419938,  0.00548705,  0.00698547,
    0.00869492,  0.01060907,  0.01271460,  0.01499087,  0.01741007,
    0.01993754,  0.02253242,  0.02514864,  0.02773610,  0.03024215,
    0.03261317,  0.03479633,  0.03674133,  0.03840211,  0.03973850,
    0.04071762,  0.04131511,  0.04151595,  0.04131511,  0.04071762,
    0.03973850,  0.03840211,  0.03674133,  0.03479633,  0.03261317,
    0.03024215,  0.02773610,  0.02514864,  0.02253242,  0.01993754,
    0.01741007,  0.01499087,  0.01271460,  0.01060907,  0.00869492,
    0.00698547,  0.00548705,  0.00419938,  0.00311633,  0.00222677,
    0.00151550,  0.00096436,  0.00055324,  0.00026107,  0.00006680,
    -0.00004981, -0.00010752, -0.00012311, -0.00011099, -0.00008306};
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
        acc += kDecimWeights[tap] * buf_[sample_idx];
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
