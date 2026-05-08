/**
 * @file madgwick_ahrs.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief 
 * @date 2026-05-07
 * 
 * 
 */
#include "madgwick_ahrs.hpp"

namespace sf::ahrs {

    Sample::Sample(const sf::protocol::DecodedImu &imu)
    {
        elapsed_time_ms = imu.elapsed_time_ms;
        time_s = imu.elapsed_time_ms * MS_TO_S;

        gyro_rad_s = {
            static_cast<double>(imu.gyro_dps[0]) * DEG_TO_RAD,
            static_cast<double>(imu.gyro_dps[1]) * DEG_TO_RAD,
            static_cast<double>(imu.gyro_dps[2]) * DEG_TO_RAD};

        accel_g = {
            static_cast<double>(imu.accel_ms2[0]) / G0,
            static_cast<double>(imu.accel_ms2[1]) / G0,
            static_cast<double>(imu.accel_ms2[2]) / G0};

        mag_uT = {
            static_cast<double>(imu.mag_uT[0]),
            static_cast<double>(imu.mag_uT[1]),
            static_cast<double>(imu.mag_uT[2])};
    }
    AHRS::AHRS(const Config &config) : config_(config) {}

    void AHRS::reset()
    {
        state_ = State{};
    }

    void AHRS::reset(math3d::Vec3 initial_gyro_bias_rad_s)
    {
        state_ = State{};
        state_.gyro_bias_rad_s = initial_gyro_bias_rad_s;
    }

    const Config &AHRS::config() const { return config_; }
    void AHRS::setConfig(const Config &config) { config_ = config; }
    const State &AHRS::state() const { return state_; }
    math3d::Quaternion AHRS::orientationEarthToImu() const { return state_.q; }
    math3d::Quaternion AHRS::orientationImuToEarth() const
    {
        return math3d::conjugate(state_.q);
    }
    math3d::Vec3 AHRS::gyroBiasRadS() const { return state_.gyro_bias_rad_s; }
    math3d::Vec3 AHRS::zeroGAcceleration() const { return state_.accel_zero_g; }
    math3d::Vec3 AHRS::globalAcceleration() const { return state_.accel_global; }
    bool AHRS::initialized() const
    {
        return state_.t_s >= config_.t_init_s;
    }

    Output AHRS::update(const sf::protocol::DecodedImu &imu)
    {
        return update(Sample(imu));
    }

} // namespace sf::ahrs
