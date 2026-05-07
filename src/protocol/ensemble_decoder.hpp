/**
 * @file ensemble_decoder.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief BLE packet decoder: transport header → typed ensemble structs.
 * @date 2026-04-30
 * 
 */

#ifndef ENSEMBLE_DECODER_HPP
#define ENSEMBLE_DECODER_HPP

#include "ensemble_types.hpp"

#include <span>
#include <vector>

namespace sf::protocol {

/**
 * @brief Parse a single raw BLE notification payload into decoded ensembles.
 *
 * Walks the transport header and ensemble records in @p packet, converting
 * each recognised record into a typed @ref DecodedEnsemble and appending it
 * to @p out. Unknown ensemble types are silently skipped.
 *
 * @param packet  Raw bytes from a single BLE notification.
 * @param out     Output vector; decoded ensembles are appended (not cleared).
 * @return        @c true on success; @c false if the transport header is
 *                malformed (packet too short to contain a valid header).
 *
 * @note Thread-safe: no shared state; all output goes through @p out.
 */
bool decode_packet(std::span<const uint8_t> packet,
                   std::vector<DecodedEnsemble>& out);

}

#endif
