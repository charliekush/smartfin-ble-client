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
    Scalar K_normal = 0.5;  ///< Normal feedback gain (~1/s).
    Scalar K_init   = 10.0; ///< Startup feedback gain (~1/s).
    Scalar t_init_s = 3.0;  ///< Startup ramp duration in seconds.

    Scalar gyro_bias_fc_hz      = 0.05;               ///< Gyro bias estimator corner frequency.
    Scalar gyro_min_rad_s       = 4.0 * DEG_TO_RAD;  ///< Stationary gyro threshold.
    Scalar gyro_stationary_time_s = 2.0;              ///< Time before bias estimator enables.

    Scalar mag_min_uT = 22.0; ///< Minimum valid Earth field magnitude.
    Scalar mag_max_uT = 67.0; ///< Maximum valid Earth field magnitude.

    Scalar accel_reject_threshold_g = 0.1;   ///< Allowed accel deviation from 1 g.
    Scalar accel_reject_time_s      = 0.100; ///< Time before accel is rejected.

    Scalar max_dt_s = 0.250; ///< Samples with a larger timestep are rejected.
};

/**
 * @brief One IMU sample converted to AHRS units.
 *
 * Constructed from a @c DecodedImu by converting:
 *   - gyro:  deg/s  → rad/s
 *   - accel: m/s²   → g
 *   - mag:   µT     → µT (no change)
 */
struct Sample {
    std::uint32_t elapsed_time_ms = 0; ///< Wire timestamp (28-bit ms field).
    double        time_s          = 0; ///< @c elapsed_time_ms in seconds.
    math3d::Vec3  gyro_rad_s{};        ///< Angular rate in rad/s.
    math3d::Vec3  accel_g{};           ///< Linear acceleration in g.
    math3d::Vec3  mag_uT{};            ///< Magnetic field in µT.

    explicit Sample(const sf::protocol::DecodedImu& imu);
};

/**
 * @brief Persistent filter state carried between updates.
 */
struct State {
    math3d::Quaternion q = math3d::Quaternion::identity(); ///< Earth-relative-to-IMU orientation.

    math3d::Vec3 gyro_bias_rad_s{}; ///< Estimated gyro bias in rad/s.

    Scalar stationary_time_s = 0.0; ///< Continuous time below gyro threshold.
    Scalar bad_accel_time_s  = 0.0; ///< Continuous time with invalid accel.
    Scalar t_s               = 0.0; ///< Time since AHRS initialization.

    std::uint32_t previous_time_ms = 0;    ///< Timestamp of the previous sample.
    bool          have_previous    = false; ///< True after first sample is processed.

    math3d::Vec3 accel_zero_g{};  ///< Gravity-removed accel, body frame.
    math3d::Vec3 accel_global{};  ///< Gravity-removed accel, ENU frame.
};

/**
 * @brief Output snapshot produced by one AHRS update.
 */
struct Output {
    bool updated = false; ///< False if the sample was skipped.

    Scalar dt_s = 0.0; ///< Timestep used for this update.
    Scalar t_s  = 0.0; ///< Filter time after this update.

    math3d::Quaternion q{};            ///< Earth-relative-to-IMU orientation.
    math3d::Quaternion imu_to_earth{}; ///< IMU-relative-to-Earth orientation.

    math3d::Vec3 gyro_bias_rad_s{}; ///< Estimated gyro bias.
    math3d::Vec3 accel_zero_g{};    ///< Body-frame zero-g acceleration.
    math3d::Vec3 accel_global{};    ///< ENU-frame zero-g acceleration.

    bool initialized       = false; ///< True after startup ramp completes.
    bool accel_used        = false; ///< True if accel contributed feedback.
    bool mag_used          = false; ///< True if mag contributed feedback.
    bool gyro_bias_updated = false; ///< True if stationary bias update ran.
};

} // namespace sf::ahrs

#endif // AHRS_TYPES_HPP
