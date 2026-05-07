/**
 * @file ble_config.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief BLE UUIDs and session defaults for the Smartfin fin.
 * @date 2026-04-30
 * 
 */

#ifndef BLE_CONFIG_HPP
#define BLE_CONFIG_HPP

#include <cstdint>

/**
 * @brief Compile-time BLE configuration constants.
 *
 * All UUID strings must match @c sf_ble_defs.hpp on the firmware side.
 */
namespace cfg {

constexpr const char* DEVICE_NAME    = "Smartfin";                             ///< Advertised peripheral name.
constexpr const char* SERVICE_UUID   = "a86d7b16-dd6c-434b-a7ee-f0ca33ac614c"; ///< Primary GATT service UUID.
constexpr const char* TELEMETRY_UUID = "deeddb00-166e-407c-8158-7b9693ad2685"; ///< Telemetry NOTIFY characteristic UUID.
constexpr const char* CONTROL_UUID   = "c39513e6-631e-439a-9b3b-affa0635b3d1"; ///< Control WRITE_WO_RSP characteristic UUID.

constexpr int      SCAN_TIMEOUT_S   = 30;  ///< BLE scan timeout in seconds.
constexpr int      RUN_DURATION_S   = 60;  ///< Maximum session streaming duration in seconds.
constexpr uint32_t TARGET_ENSEMBLES = 300; ///< Stop early when this many ensembles are decoded.

}

#endif
