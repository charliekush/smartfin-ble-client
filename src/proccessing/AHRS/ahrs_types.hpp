/**
 * @file ahrs_types.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Plain data types shared across the AHRS pipeline.
 * @date 2026-05-07
 */

#ifndef AHRS_TYPES_HPP
#define AHRS_TYPES_HPP

#include "config.hpp"
#include "constants.hpp"
#include "lin_alg.hpp"
#include "ensemble_types.hpp"

#include <cstdint>

namespace sf::ahrs {

using CFG::Scalar;

/**
 * @brief Tuning parameters for the Madgwick filter.
 *
 * All fields have sensible defaults; override only what differs from the
 * lab-tested baseline.
 */
struct Config {
    /// Normal feedback gain (~1/s).
    Scalar K_normal = 0.5;
    /// Startup feedback gain (~1/s).
    Scalar K_init = 10.0;
    /// Startup ramp duration in seconds.
    Scalar t_init_s = 3.0;

    /// Gyro bias estimator corner frequency.
    Scalar gyro_bias_fc_hz = 0.05;
    /// Stationary gyro threshold.
    Scalar gyro_min_rad_s = 4.0 * DEG_TO_RAD;
    /// Time before bias estimator enables.
    Scalar gyro_stationary_time_s = 2.0;

    /// Minimum valid Earth field magnitude.
    Scalar mag_min_uT = 22.0;
    /// Maximum valid Earth field magnitude.
    Scalar mag_max_uT = 67.0;

    /// Allowed accel deviation from 1 g.
    Scalar accel_reject_threshold_g = 0.1;
    /// Time before accel is rejected.
    Scalar accel_reject_time_s = 0.100;

    /// Samples with a larger timestep are rejected.
    Scalar max_dt_s = 0.250;
};

/**
 * @brief One IMU sample converted to AHRS units.
 *
 * Constructed from a @c DecodedImu by converting:
 *   - gyro:  deg/s  -> rad/s
 *   - accel: m/s^2  -> g
 *   - mag:   uT     -> uT (no change)
 */
struct Sample {
    /// Wire timestamp (28-bit ms field).
    std::uint32_t elapsed_time_ms = 0;
    /// @c elapsed_time_ms in seconds.
    double time_s = 0;
    /// Angular rate in rad/s.
    math3d::Vec3 gyro_rad_s{};
    /// Linear acceleration in g.
    math3d::Vec3 accel_g{};
    /// Magnetic field in uT.
    math3d::Vec3 mag_uT{};

    explicit Sample(const sf::protocol::DecodedImu& imu);
};

/**
 * @brief Persistent filter state carried between updates.
 */
struct State {
    /// Earth-relative-to-IMU orientation.
    math3d::Quaternion q = math3d::Quaternion::identity();

    /// Estimated gyro bias in rad/s.
    /// @todo Seed with lab-calibrated value; use AHRS::reset(Vec3) to apply.
    math3d::Vec3 gyro_bias_rad_s{};

    /// Continuous time below gyro threshold.
    Scalar stationary_time_s = 0.0;
    /// Continuous time with invalid accel.
    Scalar bad_accel_time_s = 0.0;
    /// Time since AHRS initialization.
    Scalar t_s = 0.0;

    /// Timestamp of the previous sample.
    std::uint32_t previous_time_ms = 0;
    /// True after first sample is processed.
    bool have_previous = false;

    /// Gravity-removed accel, body frame.
    math3d::Vec3 accel_zero_g{};
    /// Gravity-removed accel, ENU frame.
    math3d::Vec3 accel_global{};
};

/**
 * @brief Output snapshot produced by one AHRS update.
 */
struct Output {
    /// False if the sample was skipped.
    bool updated = false;

    /// Timestep used for this update.
    Scalar dt_s = 0.0;
    /// Filter time after this update.
    Scalar t_s = 0.0;

    /// Earth-relative-to-IMU orientation.
    math3d::Quaternion q{};
    /// IMU-relative-to-Earth orientation.
    math3d::Quaternion imu_to_earth{};

    /// Estimated gyro bias.
    math3d::Vec3 gyro_bias_rad_s{};
    /// Body-frame zero-g acceleration.
    math3d::Vec3 accel_zero_g{};
    /// ENU-frame zero-g acceleration.
    math3d::Vec3 accel_global{};

    /// True after startup ramp completes.
    bool initialized = false;
    /// True if accel contributed feedback.
    bool accel_used = false;
    /// True if mag contributed feedback.
    bool mag_used = false;
    /// True if stationary bias update ran.
    bool gyro_bias_updated = false;
};

} // namespace sf::ahrs

#endif // AHRS_TYPES_HPP
