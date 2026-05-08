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

    /**
     * @brief File header written at offset 0; checked on load to detect
     *        struct layout changes across build versions.
     */
    struct __attribute__((packed)) FileHeader
    {
        uint32_t magic = RIDE_FILE_MAGIC;        ///< Must equal RIDE_FILE_MAGIC.
        uint16_t version = RIDE_FILE_VERSION;    ///< Must equal RIDE_FILE_VERSION.
        uint8_t imu_size = sizeof(WireImu);      ///< sizeof(WireImu) at write time.
        uint8_t quat_size = sizeof(WireQuatImu); ///< sizeof(WireQuatImu) at write time.
        uint8_t temp_size = sizeof(WireTemp);    ///< sizeof(WireTemp) at write time.
        uint8_t fwver_size = sizeof(WireFwVer);  ///< sizeof(WireFwVer) at write time.
    };

    /**
     * @brief ISampleSink that streams decoded ensembles to a binary ride file.
     *
     * Each record is a 1-byte RecordTag followed by the matching packed wire
     * struct. The file opens with a FileHeader for version detection.
     *
     * Writes are synchronous but negligibly cheap at 55 Hz (2 KB/s).
     * The file is flushed and closed when the sink is destroyed.
     *
     * @par Usage
     * @code
     *   FileSink sink("ride_20260508.sfdat");
     *   Session session(adapter, sink, cfg);
     *   session.run();
     *   // file is flushed and closed when sink goes out of scope
     *
     *   auto data = load_ride("ride_20260508.sfdat");
     * @endcode
     */
    class FileSink final : public ISampleSink
    {
    public:
        /**
         * @brief Open a ride file for writing and write the file header.
         * @param path  Destination file path.
         * @throws std::runtime_error if the file cannot be opened.
         */
        explicit FileSink(const std::string &path)
        {
            file_ = std::fopen(path.c_str(), "wb");
            if (!file_)
                throw std::runtime_error("FileSink: cannot open " + path);
            const FileHeader hdr{};
            std::fwrite(&hdr, sizeof(hdr), 1, file_);
        }

        /**
         * @brief Flush and close the ride file.
         */
        ~FileSink() override
        {
            if (file_)
            {
                std::fflush(file_);
                std::fclose(file_);
            }
        }

        FileSink(const FileSink &) = delete;
        FileSink &operator=(const FileSink &) = delete;

        /**
         * @brief Write a temperature ensemble to the file.
         * @param s  Decoded temperature ensemble.
         */
        void on_temperature(const sf::protocol::DecodedTemp &s) override
        {
            const WireTemp w{
                s.elapsed_time_ms,
                s.temp_c,
                s.in_water ? uint8_t(1) : uint8_t(0)};
            write_record(RecordTag::Temp, &w, sizeof(w));
        }

        /**
         * @brief Write a raw IMU ensemble to the file.
         * @param s  Decoded IMU ensemble.
         */
        void on_imu(const sf::protocol::DecodedImu &s) override
        {
            WireImu w{};
            w.elapsed_time_ms = s.elapsed_time_ms;
            for (int i = 0; i < 3; ++i)
            {
                w.accel_ms2[i] = s.accel_ms2[i];
                w.gyro_dps[i] = s.gyro_dps[i];
                w.mag_uT[i] = s.mag_uT[i];
            }
            write_record(RecordTag::Imu, &w, sizeof(w));
        }

        /**
         * @brief Write a quaternion IMU ensemble to the file.
         * @param s  Decoded quaternion IMU ensemble.
         */
        void on_quat_imu(const sf::protocol::DecodedQuatImu &s) override
        {
            WireQuatImu w{};
            w.elapsed_time_ms = s.elapsed_time_ms;
            for (int i = 0; i < 3; ++i)
            {
                w.accel_ms2[i] = s.accel_ms2[i];
                w.gyro_dps[i] = s.gyro_dps[i];
                w.mag_uT[i] = s.mag_uT[i];
            }
            for (int i = 0; i < 4; ++i)
                w.q[i] = s.q[i];
            w.heading_accuracy_deg = s.heading_accuracy_deg;
            w.quat_valid = s.quat_valid ? uint8_t(1) : uint8_t(0);
            write_record(RecordTag::QuatImu, &w, sizeof(w));
        }

        /**
         * @brief Write a firmware version ensemble to the file.
         * @param s  Decoded firmware version ensemble.
         */
        void on_fw_version(const sf::protocol::DecodedFwVersion &s) override
        {
            WireFwVer w{};
            w.elapsed_time_ms = s.elapsed_time_ms;
            std::memcpy(w.version, s.version, sizeof(w.version));
            write_record(RecordTag::FwVer, &w, sizeof(w));
        }

    private:
        std::FILE *file_ = nullptr; ///< Handle to the open output file.

        /**
         * @brief Write a single tagged record to the file.
         * @param tag   Record type identifier.
         * @param data  Pointer to the packed wire struct.
         * @param size  Size of the wire struct in bytes.
         */
        void write_record(RecordTag tag, const void *data, std::size_t size)
        {
            const auto t = static_cast<uint8_t>(tag);
            std::fwrite(&t, 1, 1, file_);
            std::fwrite(data, size, 1, file_);
        }
    };

} // namespace sf::pipeline

#endif // FILE_SINK_HPP
