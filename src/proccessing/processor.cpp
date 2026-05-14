/**
 * @file processor.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Top-level processing pipeline: orient → filter → welch.
 * @date 2026-05-12
 */

#include "proccessing/processor.hpp"
#include "filter/butterworth.hpp"
#include "filter/decimator.hpp"
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
    filter(result.ride);

    // welch(result.ride);
    return result;
}

void Processor::filter(OrientedRide &ride)
{
    auto &samples = ride.samples;
    sf::filter::Decimator decim_x;
    sf::filter::Decimator decim_y;
    sf::filter::Decimator decim_z;

    std::vector<OrientedSample> decimated;
    decimated.reserve(samples.size() / sf::filter::Decimator::kFactor);

    for (const auto &sample : samples)
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        const bool has_x = decim_x.push(sample.accel_global.x, x);
        const bool has_y = decim_y.push(sample.accel_global.y, y);
        const bool has_z = decim_z.push(sample.accel_global.z, z);

        if (has_x && has_y && has_z)
        {
            OrientedSample decimated_sample = sample;
            decimated_sample.accel_global = {x, y, z};
            decimated.push_back(decimated_sample);
        }
    }

    samples = std::move(decimated);
    const size_t n = samples.size();

    if (n <= 9)
    {
        std::cerr << "[proc] ride too short after decimation, skipping filter\n";
        return;
    }

    const double duration_s =
        (samples.back().elapsed_time_ms - samples.front().elapsed_time_ms) /
        1000.0;
    if (duration_s <= 0.0)
    {
        std::cerr << "[proc] non-positive ride duration, skipping filter\n";
        return;
    }

    const double fs = (n - 1) / duration_s;
    auto hp = sf::filter::butterworth(4, CFG::bandpass_lo_hz, fs,
                                      sf::filter::FilterType::Highpass);
    auto lp = sf::filter::butterworth(4, CFG::bandpass_hi_hz, fs,
                                      sf::filter::FilterType::Lowpass);

    std::vector<double> ax(n), ay(n), az(n);
    for (size_t i = 0; i < n; ++i)
    {
        ax[i] = samples[i].accel_global.x;
        ay[i] = samples[i].accel_global.y;
        az[i] = samples[i].accel_global.z;
    }

    ax = sf::filter::filtfilt(hp, ax);
    ay = sf::filter::filtfilt(hp, ay);
    az = sf::filter::filtfilt(hp, az);

    ax = sf::filter::filtfilt(lp, ax);
    ay = sf::filter::filtfilt(lp, ay);
    az = sf::filter::filtfilt(lp, az);

    for (size_t i = 0; i < n; ++i)
    {
        samples[i].accel_global = {ax[i], ay[i], az[i]};
    }
}

} // namespace sf::proc
