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

} // namespace sf::ahrs
