/**
 * @file proc_types.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Shared types for the offline processing pipeline.
 * @date 2026-05-12
 */
#ifndef PROC_TYPES_HPP
#define PROC_TYPES_HPP

#include "protocol/ensemble_types.hpp"
#include "proccessing/math/lin_alg.hpp"

#include <cstdint>
#include <vector>

namespace sf::proc {

/**
 * @brief One IMU sample with orientation resolved to the world frame.
 *
 * Produced either by running the Madgwick AHRS on a raw IMU ensemble (12)
 * or by using the DMP quaternion from a quaternion IMU ensemble (13).
 */
struct OrientedSample {
    OrientedSample() = default;

    /**
     * @brief Construct an OrientedSample from a DMP quaternion IMU ensemble.
     *
     * Uses the on-device fused quaternion to rotate gravity-removed accel into
     * the world frame without running an additional AHRS filter.
     *
     * @param s  Decoded quaternion IMU ensemble.
     */
    explicit OrientedSample(const sf::protocol::DecodedQuatImu& s);

    std::uint32_t      elapsed_time_ms = 0;
    math3d::Quaternion q{};
    math3d::Vec3       accel_global{};
};

/**
 * @brief A time-sorted sequence of OrientedSamples from a single ride.
 *
 * Plain data holder - produced by @c Processor::process().
 */
struct OrientedRide {
    std::vector<OrientedSample> samples;
};

} // namespace sf::proc

#endif // PROC_TYPES_HPP
