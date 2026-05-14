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
    /// @brief Construct a filter with default configuration.
    AHRS() = default;

    /**
     * @brief Construct a filter with explicit configuration.
     * @param config  Tuning parameters to use for this filter instance.
     */
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
     * @return Const reference to the active tuning parameters.
     */
    const Config& config() const;

    /**
     * @brief Replace filter configuration.
     * @param config  New tuning parameters to apply.
     */
    void setConfig(const Config& config);

    /**
     * @brief Access current persistent state.
     * @return Const reference to the filter state after the last update.
     */
    const State& state() const;

    /**
     * @brief Earth-relative-to-IMU quaternion.
     * @return Current orientation quaternion q (Earth frame relative to IMU).
     */
    math3d::Quaternion orientationEarthToImu() const;

    /**
     * @brief IMU-relative-to-Earth quaternion.
     * @return Conjugate of the current orientation quaternion.
     */
    math3d::Quaternion orientationImuToEarth() const;

    /**
     * @brief Current estimated gyro bias.
     * @return Gyro bias estimate in rad/s.
     */
    math3d::Vec3 gyroBiasRadS() const;

    /**
     * @brief Body-frame zero-g acceleration.
     * @return Gravity-removed acceleration in the IMU/body frame (g).
     */
    math3d::Vec3 zeroGAcceleration() const;

    /**
     * @brief ENU-frame zero-g acceleration.
     * @return Gravity-removed acceleration rotated into the Earth ENU frame (g).
     */
    math3d::Vec3 globalAcceleration() const;

    /**
     * @brief True after startup ramp completes.
     * @return True when filter time exceeds the configured @c t_init_s.
     */
    bool initialized() const;

private:
    Config config_{}; ///< Active filter tuning parameters.
    State  state_{};  ///< Persistent state carried between updates.

    /// @brief Return the feedback gain for the current filter time.
    Scalar computeGain() const;

    /**
     * @brief Update the gyro bias estimate when the sensor is stationary.
     * @param gyro_rad_s  Raw gyro reading in rad/s.
     * @param dt_s        Timestep in seconds.
     * @param bias_updated  Set to true if the bias was updated this step.
     * @return Bias-corrected gyro rate in rad/s.
     */
    math3d::Vec3 updateGyroBias(math3d::Vec3 gyro_rad_s,
                                Scalar dt_s,
                                bool& bias_updated);

    /**
     * @brief Reject magnetometer readings outside the valid field range.
     * @param mag_uT    Raw magnetometer reading in uT.
     * @param mag_used  Set to true when the reading passes validation.
     * @return Validated (or zeroed) magnetometer vector.
     */
    math3d::Vec3 rejectMagneticDistortion(math3d::Vec3 mag_uT,
                                          bool& mag_used) const;

    /**
     * @brief Reject accelerometer readings that indicate linear acceleration.
     * @param accel_g     Raw accelerometer in g.
     * @param dt_s        Timestep in seconds.
     * @param accel_used  Set to true when the reading is accepted.
     * @return Accepted (or zeroed) accelerometer vector.
     */
    math3d::Vec3 rejectLinearAcceleration(math3d::Vec3 accel_g,
                                          Scalar dt_s,
                                          bool& accel_used);

    /**
     * @brief Compute the gradient-descent error term from the accelerometer.
     * @param q_hat        Current orientation estimate.
     * @param accel_used_g Validated accelerometer reading in g.
     * @return Error vector in body frame.
     */
    math3d::Vec3 computeAccelError(math3d::Quaternion q_hat,
                                   math3d::Vec3 accel_used_g) const;

    /**
     * @brief Compute the gradient-descent error term from the magnetometer.
     * @param q_hat       Current orientation estimate.
     * @param accel_used_g Validated accelerometer reading in g.
     * @param mag_used_uT  Validated magnetometer reading in uT.
     * @return Error vector in body frame.
     */
    math3d::Vec3 computeMagError(math3d::Quaternion q_hat,
                                 math3d::Vec3 accel_used_g,
                                 math3d::Vec3 mag_used_uT) const;

    /**
     * @brief Integrate the corrected angular rate into the orientation quaternion.
     * @param q_hat                  Current orientation estimate.
     * @param corrected_rate_rad_s   Bias-corrected, feedback-adjusted rate.
     * @param dt_s                   Timestep in seconds.
     */
    void integrateQuaternion(math3d::Quaternion q_hat,
                             math3d::Vec3 corrected_rate_rad_s,
                             Scalar dt_s);

    /**
     * @brief Remove estimated gravity and rotate acceleration to world frame.
     * @param raw_accel_g  Raw accelerometer reading in g.
     */
    void updateAccelerations(math3d::Vec3 raw_accel_g);
};

} // namespace sf::ahrs

#endif // MADGWICK_AHRS_HPP
