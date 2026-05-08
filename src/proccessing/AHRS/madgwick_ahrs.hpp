/**
 * @file madgwick_ahrs.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Madgwick AHRS filter for gyroscope drift correction.
 * @cite https://x-io.co.uk/downloads/madgwick-phd-thesis.pdf
 * @date 2026-05-03
 */

#ifndef MADGWICK_AHRS_HPP
#define MADGWICK_AHRS_HPP

#include "ahrs_types.hpp"

namespace sf::ahrs {

/**
 * @brief Madgwick gradient-descent AHRS filter.
 *
 * Fuses accelerometer, gyroscope, and magnetometer data to produce a
 * continuously corrected orientation quaternion. Gyro bias is estimated
 * during stationary periods and subtracted before integration.
 *
 * @par Usage
 * @code
 *   sf::ahrs::AHRS filter;
 *   for (auto& imu : samples)
 *       auto out = filter.update(imu);
 * @endcode
 */
class AHRS {
public:
    AHRS() = default;

    explicit AHRS(const Config& config);

    /**
     * @brief Reset all persistent filter state.
     */
    void reset();

    /**
     * @brief Reset state and seed the gyro bias estimator.
     * @param initial_gyro_bias_rad_s Lab-calibrated or previously saved bias.
     */
    void reset(math3d::Vec3 initial_gyro_bias_rad_s);

    /**
     * @brief Process one already-converted AHRS sample.
     * @param sample IMU sample in AHRS units.
     * @return Output snapshot after the update.
     */
    Output update(const Sample& sample);

    /**
     * @brief Process one decoded protocol IMU sample.
     *
     * Convenience wrapper: constructs a @c Sample then calls @c update(Sample).
     */
    Output update(const sf::protocol::DecodedImu& imu);

    /**
     * @brief Access current filter configuration.
     */
    const Config& config() const;

    /**
     * @brief Replace filter configuration.
     */
    void setConfig(const Config& config);

    /**
     * @brief Access current persistent state.
     */
    const State& state() const;

    /// @brief Earth-relative-to-IMU quaternion.
    math3d::Quaternion orientationEarthToImu() const;

    /// @brief IMU-relative-to-Earth quaternion.
    math3d::Quaternion orientationImuToEarth() const;

    /// @brief Current estimated gyro bias.
    math3d::Vec3 gyroBiasRadS() const;

    /// @brief Body-frame zero-g acceleration.
    math3d::Vec3 zeroGAcceleration() const;

    /// @brief ENU-frame zero-g acceleration.
    math3d::Vec3 globalAcceleration() const;

    /// @brief True after startup ramp completes.
    bool initialized() const;

private:
    Config config_{};
    State  state_{};

    Scalar computeGain() const;

    math3d::Vec3 updateGyroBias(math3d::Vec3 gyro_rad_s, Scalar dt_s, bool& bias_updated);
    math3d::Vec3 rejectMagneticDistortion(math3d::Vec3 mag_uT, bool& mag_used) const;
    math3d::Vec3 rejectLinearAcceleration(math3d::Vec3 accel_g, Scalar dt_s, bool& accel_used);
    math3d::Vec3 computeAccelError(math3d::Quaternion q_hat, math3d::Vec3 accel_used_g) const;
    math3d::Vec3 computeMagError(math3d::Quaternion q_hat, math3d::Vec3 accel_used_g, math3d::Vec3 mag_used_uT) const;
    void         integrateQuaternion(math3d::Quaternion q_hat, math3d::Vec3 corrected_rate_rad_s, Scalar dt_s);
    void         updateAccelerations(math3d::Vec3 raw_accel_g);
};

} // namespace sf::ahrs

#endif // MADGWICK_AHRS_HPP
