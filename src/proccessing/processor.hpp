/**
 * @file processor.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Stateful ride-processing pipeline.
 * @date 2026-05-12
 */
#ifndef PROCESSOR_HPP
#define PROCESSOR_HPP

#include "proccessing/proc_types.hpp"
#include "proccessing/AHRS/ahrs_types.hpp"
#include "pipeline/file_sink.hpp"

#include <vector>

namespace sf::proc {

/**
 * @brief Configuration for the full processing pipeline.
 *
 * Aggregates per-stage config structs so callers construct one object and pass
 * it to Processor. Add new stage configs here as the pipeline grows.
 */
struct ProcessorConfig {
    sf::ahrs::Config ahrs{};
};

/**
 * @brief Stateful offline processing pipeline.
 *
 * Owns pipeline configuration and converts raw @c RideData into a fully
 * oriented, time-sorted @c OrientedRide. Construct once with a
 * @c ProcessorConfig, then call @c process() for each ride.
 *
 * @par Usage
 * @code
 *   sf::proc::Processor proc;                 // default config
 *   OrientedRide ride = proc.process(data);
 * @endcode
 */
class Processor {
public:
    /// @brief Construct with default pipeline configuration.
    Processor() = default;

    /// @brief Construct with explicit pipeline configuration.
    explicit Processor(const ProcessorConfig& cfg);

    /**
     * @brief Run the full proccessing pipeline on one ride.
     *
     * Steps performed in order:
     *  1. Sort @c data.imu and @c data.quat_imu by timestamp (timsort).
     *  2. Convert raw IMU samples to @c OrientedSample via the Madgwick AHRS.
     *  3. Convert quaternion-IMU samples to @c OrientedSample directly.
     *  4. Merge both sorted sequences into a single time-sorted result.
     *
     * @param data  Ride data loaded from a .sfdat file. Sorted in place.
     * @return      Time-sorted @c OrientedRide.
     */
    OrientedRide process(sf::pipeline::RideData &data);

private:
    ProcessorConfig cfg_{};

    std::vector<OrientedSample> orient_from_imu(
        const std::vector<sf::protocol::DecodedImu>& samples);

    std::vector<OrientedSample> orient_from_quat_imu(
        const std::vector<sf::protocol::DecodedQuatImu>& samples);
};

} // namespace sf::proc

#endif // PROCESSOR_HPP
