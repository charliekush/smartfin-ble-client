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
#include "pipeline/file_sink.hpp"

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

    explicit OrientedSample(const sf::protocol::DecodedQuatImu& s);

    std::uint32_t      elapsed_time_ms = 0;
    math3d::Quaternion q{};
    math3d::Vec3       accel_global{};
};

/**
 * @brief A time-series of OrientedSamples from a single ride.
 */
struct OrientedRide {
    std::vector<OrientedSample> samples;
    /// build an OrientedRide from decoded IMU values
    explicit OrientedRide(sf::pipeline::RideData &data);

    private:

        /**
         * @brief Run the Madgwick AHRS on RideData::imu and return an OrientedRide.
         *
         * Processes each DecodedImu sample through a fresh AHRS filter. Samples
         * skipped by the filter on its first step (no valid dt yet) are omitted.
         *
         * @param data  Ride data loaded from a .sfdat file.
         * @return OrientedRide with one sample per accepted AHRS update.
         */
        std::vector<OrientedSample> orient_from_imu(
            const std::vector<sf::protocol::DecodedImu> &decoded_samples);
        /**
         * @brief Build an OrientedRide from quat_imu using DMP quaternions
         *
         * @param data  Ride data loaded from a .sfdat file.
         * @return OrientedRide with one sample per quat_imu record.
         */
        std::vector<OrientedSample> orient_from_quat_imu(
            const std::vector<sf::protocol::DecodedQuatImu> &decoded_samples);


        

};

} // namespace sf::proc

#endif // PROC_TYPES_HPP
