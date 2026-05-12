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
     * @brief Ensemble type identifiers as defined in the firmware wire protocol.
     */
    enum class EnsembleId : ::std::uint8_t
    {
        Temp = 0x01,            ///< temperature only
        RawHighRateImu = 0x0C,  ///< raw IMU payload
        QuatHighRateImu = 0x0D, ///<  IMU payload w/ quaternion
        Text = 0x0F,            ///< ASCII text payload — SS_fwVerFunc.
    };

    /**
     * @brief Decoded payload for ENS_TEMP (0x01), Ensemble01_data_t.
     */
    struct DecodedTemp
    {
        ::std::uint32_t elapsed_time_ms; ///< Milliseconds since session start.
        float temp_c;             ///< Water temperature in degrees Celsius.
        bool in_water;            ///< True when the fin detects water immersion.
    };

/**
 * @brief Decoded payload for ENS_TEMP_HIGH_DATA_RATE_IMU (0x0C), Ensemble12_data_t.
 *
 * Fixed-point wire values are scaled to physical units on decode:
 * Q14 -> m/s^2, Q7 -> deg/s, Q3 -> uT.
 */
struct DecodedImu {
    ::std::uint32_t elapsed_time_ms; ///< Milliseconds since session start.
    float accel_ms2[3];       ///< Linear acceleration in m/s^2 [x, y, z].
    float gyro_dps[3];        ///< Angular rate in deg/s [x, y, z].
    float mag_uT[3];          ///< Magnetic field in uT [x, y, z].
};
/**
 * @brief Decoded payload for ENS_QUAT_HIGH_DATA_RATE_IMU (0x0D).
 *
 * Extends DecodedImu with a unit quaternion fused on the device and a
 * heading accuracy estimate. Q30 fixed-point wire values are scaled to
 * [-1, 1] on decode; q0 is reconstructed from q1/q2/q3.
 */
struct DecodedQuatImu
{
    ::std::uint32_t elapsed_time_ms; ///< Milliseconds since session start.
    float accel_ms2[3];       ///< Linear acceleration in m/s^2 [x, y, z].
    float gyro_dps[3];        ///< Angular rate in deg/s [x, y, z].
    float mag_uT[3];          ///< Magnetic field in uT [x, y, z].

    float q[4];                  ///< Unit quaternion [q0, q1, q2, q3].
    float heading_accuracy_deg;  ///< Estimated heading accuracy in degrees.
    bool  quat_valid;            ///< False when accuracy exceeds the validity threshold.
};

/**
 * @brief Decoded payload for ENS_TEXT (0x0F) — SS_fwVerFunc.
 *
 * Wire layout: uint8_t nChars, char verBuf[nChars] (no null terminator).
 * Format: "FW<major>.<minor>.<build><branch>", max 32 chars.
 */
struct DecodedFwVersion {
    ::std::uint32_t elapsed_time_ms; ///< Milliseconds since session start.
    char     version[33]; ///< Null-terminated version string, max 32 wire chars.
};

/// @brief Variant holding any single decoded ensemble from the wire protocol.
using DecodedEnsemble = std::variant<
    DecodedTemp,
    DecodedImu,
    DecodedQuatImu,
    DecodedFwVersion
>;

} // namespace sf::protocol

#endif
