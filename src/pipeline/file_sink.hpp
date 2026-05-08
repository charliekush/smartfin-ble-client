/**
 * @file file_sink.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief ISampleSink that writes decoded ensembles to a binary ride file,
 *        and load_ride() to unpack that file back into decoded structs.
 * @date 2026-05-08
 */

#ifndef FILE_SINK_HPP
#define FILE_SINK_HPP

#include "sample_sink.hpp"
#include "protocol/ensemble_types.hpp"

#include <cstdio>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace sf::pipeline {

    /// @brief Magic number at the start of every .sfdat file ("SFDAT").
    static constexpr uint32_t RIDE_FILE_MAGIC = 0x5346444154u;
    /// @brief Current file format version; bumped on breaking layout changes.
    static constexpr uint16_t RIDE_FILE_VERSION = 1;

    /**
     * @brief Tag byte prepended to each record to identify its wire type.
     */
    enum class RecordTag : uint8_t
    {
        Imu = 0x01,     ///< Raw IMU record (WireImu).
        QuatImu = 0x02, ///< Quaternion IMU record (WireQuatImu).
        Temp = 0x03,    ///< Temperature record (WireTemp).
        FwVer = 0x04,   ///< Firmware version record (WireFwVer).
    };

    /**
     * @brief Packed on-disk layout for a raw IMU ensemble.
     */
    struct __attribute__((packed)) WireImu
    {
        uint32_t elapsed_time_ms; ///< Milliseconds since session start.
        float accel_ms2[3];       ///< Linear acceleration in m/s^2 [x, y, z].
        float gyro_dps[3];        ///< Angular rate in deg/s [x, y, z].
        float mag_uT[3];          ///< Magnetic field in uT [x, y, z].
    };

    /**
     * @brief Packed on-disk layout for a quaternion IMU ensemble.
     */
    struct __attribute__((packed)) WireQuatImu
    {
        uint32_t elapsed_time_ms;   ///< Milliseconds since session start.
        float accel_ms2[3];         ///< Linear acceleration in m/s^2 [x, y, z].
        float gyro_dps[3];          ///< Angular rate in deg/s [x, y, z].
        float mag_uT[3];            ///< Magnetic field in uT [x, y, z].
        float q[4];                 ///< Quaternion [q0, q1, q2, q3].
        float heading_accuracy_deg; ///< Estimated heading accuracy in degrees.
        uint8_t quat_valid;         ///< Non-zero when accuracy is within threshold.
    };

    /**
     * @brief Packed on-disk layout for a temperature ensemble.
     */
    struct __attribute__((packed)) WireTemp
    {
        uint32_t elapsed_time_ms; ///< Milliseconds since session start.
        float temp_c;             ///< Water temperature in degrees Celsius.
        uint8_t in_water;         ///< Non-zero when the fin detects water immersion.
    };

    /**
     * @brief Packed on-disk layout for a firmware version ensemble.
     */
    struct __attribute__((packed)) WireFwVer
    {
        uint32_t elapsed_time_ms; ///< Milliseconds since session start.
        char version[33];         ///< Null-terminated version string, max 32 chars.
    };

} // namespace sf::pipeline

#endif // FILE_SINK_HPP
