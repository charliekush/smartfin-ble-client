/**
 * @file processor.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Processor::process and orientation helpers.
 * @date 2026-05-12
 */

#include "proccessing/processor.hpp"
#include "proccessing/AHRS/madgwick_ahrs.hpp"
#include "proccessing/sort/timsort.hpp"

#include <algorithm>

namespace sf::proc {

Processor::Processor(const ProcessorConfig& cfg) : cfg_(cfg) {}

OrientedRide Processor::process(sf::pipeline::RideData& data)
{
    auto by_time = [](const auto& a, const auto& b) {
        return a.elapsed_time_ms < b.elapsed_time_ms;
    };

    timsort(data.imu, by_time);
    timsort(data.quat_imu, by_time);

    std::vector<OrientedSample> imu      = orient_from_imu(data.imu);
    std::vector<OrientedSample> quat_imu = orient_from_quat_imu(data.quat_imu);

    auto by_sample_time = [](const OrientedSample& a, const OrientedSample& b) {
        return a.elapsed_time_ms < b.elapsed_time_ms;
    };

    OrientedRide ride;
    ride.samples.resize(imu.size() + quat_imu.size());
    std::merge(imu.begin(),      imu.end(),
               quat_imu.begin(), quat_imu.end(),
               ride.samples.begin(),
               by_sample_time);

    return ride;
}

std::vector<OrientedSample> Processor::orient_from_imu(
    const std::vector<sf::protocol::DecodedImu>& decoded_samples)
{
    sf::ahrs::AHRS filter(cfg_.ahrs);
    std::vector<OrientedSample> out;
    out.reserve(decoded_samples.size());

    for (const auto& imu : decoded_samples) {
        const auto result = filter.update(imu);
        if (!result.updated)
            continue;

        OrientedSample s;
        s.elapsed_time_ms = imu.elapsed_time_ms;
        s.q               = result.q;
        s.accel_global    = result.accel_global;
        out.emplace_back(s);
    }

    return out;
}

std::vector<OrientedSample> Processor::orient_from_quat_imu(
    const std::vector<sf::protocol::DecodedQuatImu>& decoded_samples)
{
    std::vector<OrientedSample> out;
    out.reserve(decoded_samples.size());

    for (const auto& q_imu : decoded_samples)
        out.emplace_back(q_imu);

    return out;
}

} // namespace sf::proc
