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
#include "filter/butterworth.hpp"
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
 * @brief Output of the full processing pipeline for one ride.
 */
struct ProcessedRide
{
    OrientedRide ride; ///< Time-sorted, world-frame oriented samples.
    // wave metrics, PSD, etc. added here as pipeline stages are implemented
};

/**
 * @brief Stateful offline processing pipeline.
 *
 * Owns pipeline configuration and converts raw @c RideData into a
 * @c ProcessedRide. Construct once with a @c ProcessorConfig, then call
 * @c process() for each ride.
 *
 * @par Usage
 * @code
 *   sf::proc::Processor proc;
 *   ProcessedRide result = proc.process(data);
 * @endcode
 */
class Processor {
public:
    /// @brief Construct with default pipeline configuration.
    Processor() = default;

    /// @brief Construct with explicit pipeline configuration.
    explicit Processor(const ProcessorConfig& cfg);

    /**
     * @brief Run the full processing pipeline on one ride.
     *
     * Orchestrates orient → filter → welch in order.
     *
     * @param data  Ride data loaded from a .sfdat file. Sorted in place.
     * @return      Fully processed ride result.
     */
    ProcessedRide process(sf::pipeline::RideData &data);

    /**
     * @brief Sort and orient raw IMU/quat-IMU samples into world frame.
     *
     * @param data  Ride data. Sorted in place by timestamp.
     * @return      Time-sorted @c OrientedRide.
     */
    OrientedRide orient_ride(sf::pipeline::RideData &data);

private:
    ProcessorConfig cfg_{};

    std::vector<OrientedSample> orient_from_imu(
        const std::vector<sf::protocol::DecodedImu>& samples);

    std::vector<OrientedSample> orient_from_quat_imu(
        const std::vector<sf::protocol::DecodedQuatImu>& samples);

    /// @brief Apply filtfilt with @p coeffs to each axis of accel_global.
    void filter(OrientedRide &ride, const sf::filter::ButterworthCoeffs &coeffs);
};

} // namespace sf::proc

#endif // PROCESSOR_HPP
