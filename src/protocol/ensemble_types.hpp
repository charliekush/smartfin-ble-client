/**
 * @file ensemble_types.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Decoded ensemble structs and the EnsembleId enum mirroring firmware.
 * @date 2026-04-30
 * 
 */

#ifndef ENSEMBLE_TYPES_HPP
#define ENSEMBLE_TYPES_HPP

#include <cstdint>
#include <variant>

namespace sf::protocol {

/**
 * @brief Mirror of firmware EnsembleID_e (ensembleTypes.hpp).
 *
 * Only types the client actively decodes are listed; others remain as named
 * constants so unknown IDs can be logged meaningfully.
 */
enum class EnsembleId : uint8_t {
    Temp               = 0x01, ///< Temperature only.
    Acc                = 0x02, ///< Accelerometer only.
    Gps                = 0x03, ///< GPS only.
    TempAcc            = 0x04, ///< Temperature and accelerometer.
    TempGps            = 0x05, ///< Temperature and GPS.
    TempAccGps         = 0x06, ///< Temperature, accelerometer, and GPS.
    Batt               = 0x07, ///< Battery voltage.
    TempTime           = 0x08, ///< Temperature and timestamp.
    Imu                = 0x09, ///< IMU sample with accel, gyro, and magnetometer.
    TempImu            = 0x0A, ///< Temperature and IMU.
    TempImuGps         = 0x0B, ///< Temperature, IMU, and GPS.
    TempHighRateImu    = 0x0C, ///< High-rate IMU payload used by Ensemble12.
    Text               = 0x0F, ///< ASCII text payload.
};

/**
 * @brief Decoded payload for ENS_TEMP (0x01), Ensemble01_data_t.
 *
 * All physical units are post-scaling; downstream code can
 * use these directly without knowing the wire encoding.
 */
struct DecodedTemp {
    uint32_t elapsed_time_ds; ///< Deciseconds since session start.
    float    temp_c;          ///< Water temperature in degrees Celsius.
    bool     in_water;        ///< True when the fin detects water immersion.
};

/**
 * @brief Decoded payload for ENS_TEMP_HIGH_DATA_RATE_IMU (0x0C), Ensemble12_data_t.
 *
 * All vectors are ordered [x, y, z]. Fixed-point wire values are scaled to
 * physical units on decode: Q14 → m/s^2, Q7 → deg/s, Q3 → µT.
 */
struct DecodedImu {
    uint32_t elapsed_time_ds; ///< Deciseconds since session start.
    float    accel_ms2[3];    ///< Linear acceleration in meters per second squared.
    float    gyro_dps[3];     ///< Angular rate in degrees per second.
    float    mag_uT[3];       ///< Magnetic field in microtesla.
};

/**
 * @brief Variant holding any one decoded ensemble type.
 *
 * Add new decoded structs here and to ISampleSink when new ensemble types
 * are supported.
 */
using DecodedEnsemble = std::variant<DecodedTemp, DecodedImu>;

}

#endif
