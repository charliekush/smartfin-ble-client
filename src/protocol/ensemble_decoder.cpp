/**
 * @file ensemble_decoder.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief BLE packet decoder implementation.
 * @date 2026-04-30
 * 
 */

#include "ensemble_decoder.hpp"

#include <cstring>
#include <span>

namespace sf::protocol {

/**
 * @brief Size in bytes of the Smartfin transport header.
 */
static constexpr size_t TRANSPORT_HEADER_SIZE = 6;

/**
 * @brief Size in bytes of the packed ensemble header.
 */
static constexpr size_t ENSEMBLE_HEADER_SIZE = 3;

/**
 * @brief Size in bytes of an ENS_TEMP payload.
 */
static constexpr size_t TEMP_PAYLOAD_SIZE = 3;

/**
 * @brief Size in bytes of an ENS_TEMP_HIGH_DATA_RATE_IMU payload.
 */
static constexpr size_t IMU_PAYLOAD_SIZE  = 18;

/**
 * @brief Read a little-endian unsigned 16-bit integer.
 * @param p Pointer to the first byte.
 * @return Decoded unsigned 16-bit value.
 */
static inline uint16_t read_u16_le(const uint8_t* p) {
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

/**
 * @brief Read a little-endian signed 16-bit integer.
 * @param p Pointer to the first byte.
 * @return Decoded signed 16-bit value.
 */
static inline int16_t read_i16_le(const uint8_t* p) {
    return static_cast<int16_t>(read_u16_le(p));
}

/**
 * @brief Parse the packed ensemble header fields.
 * @param p Pointer to the first header byte.
 * @param ens_type Output ensemble type identifier.
 * @param elapsed_ds Output elapsed time in deciseconds.
 */
static void parse_ensemble_header(const uint8_t* p,
                                  uint8_t& ens_type,
                                  uint32_t& elapsed_ds) {
    uint32_t word = static_cast<uint32_t>(p[0])
                  | (static_cast<uint32_t>(p[1]) << 8)
                  | (static_cast<uint32_t>(p[2]) << 16);
    ens_type   = static_cast<uint8_t>(word & 0x0Fu);
    elapsed_ds = (word >> 4) & 0x000FFFFFu;
}

bool decode_packet(std::span<const uint8_t> packet,
                   std::vector<DecodedEnsemble>& out) {
    if (packet.size() < TRANSPORT_HEADER_SIZE) {
        return false;
    }

    /// Transport header layout is version, packet type, sequence, then payload length.
    const uint16_t payload_len = read_u16_le(packet.data() + 4);
    const size_t payload_end   = TRANSPORT_HEADER_SIZE
                               + static_cast<size_t>(payload_len);

    /// Clamp to the actual buffer size so malformed lengths cannot read out of bounds.
    const size_t end = std::min(payload_end, packet.size());

    size_t offset = TRANSPORT_HEADER_SIZE;

    while (offset + ENSEMBLE_HEADER_SIZE <= end) {
        uint8_t  ens_type;
        uint32_t elapsed_ds;
        parse_ensemble_header(packet.data() + offset, ens_type, elapsed_ds);

        if (ens_type == static_cast<uint8_t>(EnsembleId::Temp)) {
            const size_t record_size = ENSEMBLE_HEADER_SIZE + TEMP_PAYLOAD_SIZE;
            if (offset + record_size > end) break;

            const uint8_t* payload    = packet.data() + offset + ENSEMBLE_HEADER_SIZE;
            const int16_t  scaled_temp = read_i16_le(payload);
            const bool     in_water    = payload[2] != 0;

            out.push_back(DecodedTemp{
                .elapsed_time_ds = elapsed_ds,
                .temp_c          = scaled_temp / 128.0f,
                .in_water        = in_water,
            });

            offset += record_size;
            continue;
        }

        if (ens_type == static_cast<uint8_t>(EnsembleId::TempHighRateImu)) {
            /// Ensemble12_data_t stores 3 accel, 3 gyro, and 3 magnetometer samples.
            const size_t record_size = ENSEMBLE_HEADER_SIZE + IMU_PAYLOAD_SIZE;
            if (offset + record_size > end) break;

            const uint8_t* payload = packet.data() + offset + ENSEMBLE_HEADER_SIZE;

            DecodedImu imu{};
            imu.elapsed_time_ds = elapsed_ds;

            for (int i = 0; i < 3; ++i) {
                imu.accel_ms2[i] = read_i16_le(payload + i * 2)       / 16384.0f;
                imu.gyro_dps[i]  = read_i16_le(payload + 6 + i * 2)   / 128.0f;
                imu.mag_uT[i]    = read_i16_le(payload + 12 + i * 2)  / 8.0f;
            }

            out.push_back(imu);
            offset += record_size;
            continue;
        }

        /// Stop parsing when an unknown ensemble type is encountered.
        break;
    }

    return true;
}

}
