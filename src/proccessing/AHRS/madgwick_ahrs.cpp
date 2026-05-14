/**
 * @file madgwick_ahrs.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Madgwick AHRS filter implementation: sample conversion, update loop,
 * and accessors.
 * @date 2026-05-07
 */
#include "madgwick_ahrs.hpp"

namespace sf::ahrs
{

Sample::Sample(const sf::protocol::DecodedImu &imu)
{
    elapsed_time_ms = imu.elapsed_time_ms;
    time_s = imu.elapsed_time_ms * MS_TO_S;

    gyro_rad_s = {static_cast<double>(imu.gyro_dps[0]) * DEG_TO_RAD,
                  static_cast<double>(imu.gyro_dps[1]) * DEG_TO_RAD,
                  static_cast<double>(imu.gyro_dps[2]) * DEG_TO_RAD};

    accel_g = {static_cast<double>(imu.accel_ms2[0]) / G0,
               static_cast<double>(imu.accel_ms2[1]) / G0,
               static_cast<double>(imu.accel_ms2[2]) / G0};

    mag_uT = {static_cast<double>(imu.mag_uT[0]),
              static_cast<double>(imu.mag_uT[1]),
              static_cast<double>(imu.mag_uT[2])};
}
AHRS::AHRS(const Config &config) : config_(config)
{
}

void AHRS::reset()
{
    state_ = State{};
}

void AHRS::reset(math3d::Vec3 initial_gyro_bias_rad_s)
{
    state_ = State{};
    state_.gyro_bias_rad_s = initial_gyro_bias_rad_s;
}

const Config &AHRS::config() const
{
    return config_;
}
void AHRS::setConfig(const Config &config)
{
    config_ = config;
}
const State &AHRS::state() const
{
    return state_;
}
math3d::Quaternion AHRS::orientationEarthToImu() const
{
    return state_.q;
}
math3d::Quaternion AHRS::orientationImuToEarth() const
{
    return math3d::conjugate(state_.q);
}
math3d::Vec3 AHRS::gyroBiasRadS() const
{
    return state_.gyro_bias_rad_s;
}
math3d::Vec3 AHRS::zeroGAcceleration() const
{
    return state_.accel_zero_g;
}
math3d::Vec3 AHRS::globalAcceleration() const
{
    return state_.accel_global;
}
bool AHRS::initialized() const
{
    return state_.t_s >= config_.t_init_s;
}

Output AHRS::update(const sf::protocol::DecodedImu &imu)
{
    return update(Sample(imu));
}

Output AHRS::update(const Sample &sample)
{
    Output out;

    // First sample: record timestamp and return without integrating
    if (!state_.have_previous)
    {
        state_.previous_time_ms = sample.elapsed_time_ms;
        state_.have_previous = true;
        return out;
    }

    // Compute dt_s from ms timestamps to avoid accumulating float error
    const Scalar dt_s =
        static_cast<Scalar>(sample.elapsed_time_ms - state_.previous_time_ms) *
        MS_TO_S;
    state_.previous_time_ms = sample.elapsed_time_ms;

    if (dt_s <= 0.0 || !std::isfinite(dt_s) || dt_s > config_.max_dt_s)
        return out;

    state_.t_s += dt_s;

    // Normalized quaternion
    const math3d::Quaternion q_hat = math3d::normalize(state_.q);

    // Ramp gain from K_init down to K_normal during startup (Eq. 7.6)
    const Scalar K = computeGain();

    // Gyroscope bias compensation (Eq. 7.7-7.9)
    bool bias_updated = false;
    const math3d::Vec3 corrected_gyro =
        updateGyroBias(sample.gyro_rad_s, dt_s, bias_updated);

    // Magnetic distortion rejection (Eq. 7.10)
    bool mag_used = false;
    const math3d::Vec3 mag_used_uT =
        rejectMagneticDistortion(sample.mag_uT, mag_used);

    // Linear acceleration rejection (Eq. 7.11)
    bool accel_used = false;
    const math3d::Vec3 accel_used_g =
        rejectLinearAcceleration(sample.accel_g, dt_s, accel_used);

    // Accelerometer feedback error (Eq. 7.3)
    const math3d::Vec3 ea = computeAccelError(q_hat, accel_used_g);

    // Magnetometer feedback error (Eq. 7.4)
    const math3d::Vec3 em = computeMagError(q_hat, accel_used_g, mag_used_uT);

    // Combine feedback error (Eq. 7.5)
    math3d::Vec3 e{};
    if (math3d::isNonzero(accel_used_g) && math3d::isNonzero(mag_used_uT))
        e = ea + em;
    else if (math3d::isNonzero(accel_used_g))
        e = ea;

    // Correct gyro, integrate, and renormalize (Eq. 7.1-7.2)
    integrateQuaternion(q_hat, corrected_gyro - K * e, dt_s);

    // Zero-g and global acceleration (Eq. 7.12-7.13)
    updateAccelerations(sample.accel_g);

    out.updated = true;
    out.dt_s = dt_s;
    out.t_s = state_.t_s;
    out.q = state_.q;
    out.imu_to_earth = math3d::conjugate(state_.q);
    out.gyro_bias_rad_s = state_.gyro_bias_rad_s;
    out.accel_zero_g = state_.accel_zero_g;
    out.accel_global = state_.accel_global;
    out.initialized = initialized();
    out.accel_used = accel_used;
    out.mag_used = mag_used;
    out.gyro_bias_updated = bias_updated;

    return out;
}

Scalar AHRS::computeGain() const
{
    if (state_.t_s >= config_.t_init_s)
        return config_.K_normal;

    const Scalar ramp = (config_.t_init_s - state_.t_s) / config_.t_init_s;
    return config_.K_normal + ramp * (config_.K_init - config_.K_normal);
}

math3d::Vec3 AHRS::updateGyroBias(math3d::Vec3 gyro_rad_s, Scalar dt_s,
                                  bool &bias_updated)
{
    // Stationarity detection: all axes must stay below the threshold
    if (std::abs(gyro_rad_s.x) < config_.gyro_min_rad_s &&
        std::abs(gyro_rad_s.y) < config_.gyro_min_rad_s &&
        std::abs(gyro_rad_s.z) < config_.gyro_min_rad_s)
        state_.stationary_time_s += dt_s;
    else
        state_.stationary_time_s = 0.0;

    bias_updated = false;
    if (state_.stationary_time_s > config_.gyro_stationary_time_s)
    {
        // First order low-pass: w_bias += alpha * (w - w_bias)  (Eq. 7.8)
        const Scalar alpha =
            2.0 * std::numbers::pi * config_.gyro_bias_fc_hz * dt_s;
        state_.gyro_bias_rad_s += alpha * (gyro_rad_s - state_.gyro_bias_rad_s);
        bias_updated = true;
    }

    return gyro_rad_s - state_.gyro_bias_rad_s;
}

math3d::Vec3 AHRS::rejectMagneticDistortion(math3d::Vec3 mag_uT,
                                            bool &mag_used) const
{
    const Scalar n = math3d::norm(mag_uT);
    if (n > config_.mag_min_uT && n < config_.mag_max_uT)
    {
        mag_used = true;
        return mag_uT;
    }
    mag_used = false;
    return {};
}

math3d::Vec3 AHRS::rejectLinearAcceleration(math3d::Vec3 accel_g, Scalar dt_s,
                                            bool &accel_used)
{
    const Scalar deviation = std::abs(math3d::norm(accel_g) - 1.0);
    if (deviation > config_.accel_reject_threshold_g)
        state_.bad_accel_time_s += dt_s;
    else
        state_.bad_accel_time_s = 0.0;

    if (state_.bad_accel_time_s > config_.accel_reject_time_s)
    {
        accel_used = false;
        return {};
    }
    accel_used = true;
    return accel_g;
}

math3d::Vec3 AHRS::computeAccelError(math3d::Quaternion q_hat,
                                     math3d::Vec3 accel_used_g) const
{
    if (!math3d::isNonzero(accel_used_g))
        return {};

    const math3d::Vec3 a_hat = math3d::normalize(accel_used_g);
    const math3d::Vec3 gravity_est = math3d::gravityFromQuaternion(q_hat);
    return math3d::cross(a_hat, gravity_est);
}

math3d::Vec3 AHRS::computeMagError(math3d::Quaternion q_hat,
                                   math3d::Vec3 accel_used_g,
                                   math3d::Vec3 mag_used_uT) const
{
    if (!math3d::isNonzero(accel_used_g) || !math3d::isNonzero(mag_used_uT))
        return {};

    const math3d::Vec3 a_hat = math3d::normalize(accel_used_g);
    const math3d::Vec3 m_hat = math3d::normalize(mag_used_uT);

    // cross(a_hat, m_hat) is the measured East direction (tilt-compensated)
    math3d::Vec3 measured_east = math3d::cross(a_hat, m_hat);
    if (!math3d::isNonzero(measured_east))
        return {};
    measured_east = math3d::normalize(measured_east);

    const math3d::Vec3 east_est = math3d::eastFromQuaternion(q_hat);
    return math3d::cross(measured_east, east_est);
}

void AHRS::integrateQuaternion(math3d::Quaternion q_hat,
                               math3d::Vec3 corrected_rate_rad_s, Scalar dt_s)
{
    // q_dot = 0.5 * q_hat otimes [0, w']  (Eq. 7.2)
    const math3d::Quaternion q_dot =
        0.5 * (q_hat * math3d::Quaternion::pure(corrected_rate_rad_s));

    state_.q = math3d::normalize(state_.q + q_dot * dt_s);
}

void AHRS::updateAccelerations(math3d::Vec3 raw_accel_g)
{
    // Zero-g: remove the 1-g gravity component from body frame (Eq. 7.12)
    state_.accel_zero_g = raw_accel_g - math3d::gravityFromQuaternion(state_.q);

    // Global: rotate zero-g accel into the ENU Earth frame (Eq. 7.13)
    const math3d::Quaternion rotated =
        state_.q * math3d::Quaternion::pure(state_.accel_zero_g) *
        math3d::conjugate(state_.q);

    state_.accel_global = rotated.vector();
}
} // namespace sf::ahrs
