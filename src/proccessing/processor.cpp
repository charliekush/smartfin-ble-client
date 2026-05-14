/**
 * @file processor.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Top-level processing pipeline: orient → filter → welch.
 * @date 2026-05-12
 */

#include "proccessing/processor.hpp"
#include "filter/butterworth.hpp"
#include "filter/filtfilt.hpp"
#include "proccessing/config.hpp"

#include <iostream>
#include <vector>

namespace sf::proc {

Processor::Processor(const ProcessorConfig& cfg) : cfg_(cfg) {}

ProcessedRide Processor::process(sf::pipeline::RideData &data)
{
    ProcessedRide result;
    result.ride = orient_ride(data);

    const auto &samples = result.ride.samples;
    if (samples.size() < 3)
    {
        std::cerr << "[proc] ride has fewer than 3 samples, skipping filter\n";
        return result;
    }
    double fs =
        (samples.size() - 1) /
        ((samples.back().elapsed_time_ms - samples.front().elapsed_time_ms) /
         1000.0);
    auto hp = sf::filter::butterworth(4, CFG::bandpass_lo_hz, fs,
                                      sf::filter::FilterType::Highpass);
    auto lp = sf::filter::butterworth(4, CFG::bandpass_hi_hz, fs,
                                      sf::filter::FilterType::Lowpass);
    filter(result.ride, hp);
    filter(result.ride, lp);

    // welch(result.ride);
    return result;
}

void Processor::filter(OrientedRide &ride,
                       const sf::filter::ButterworthCoeffs &coeffs)
{
    auto &samples = ride.samples;
    const size_t n = samples.size();

    std::vector<double> ax(n), ay(n), az(n);
    for (size_t i = 0; i < n; ++i)
    {
        ax[i] = samples[i].accel_global.x;
        ay[i] = samples[i].accel_global.y;
        az[i] = samples[i].accel_global.z;
    }

    ax = sf::filter::filtfilt(coeffs, ax);
    ay = sf::filter::filtfilt(coeffs, ay);
    az = sf::filter::filtfilt(coeffs, az);

    for (size_t i = 0; i < n; ++i)
    {
        samples[i].accel_global = {ax[i], ay[i], az[i]};
    }
}

} // namespace sf::proc
