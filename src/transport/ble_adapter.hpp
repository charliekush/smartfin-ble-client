/**
 * @file ble_adapter.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Abstract BLE peripheral adapter interface.
 * @date 2026-04-30
 * 
 */

#ifndef BLE_ADAPTER_HPP
#define BLE_ADAPTER_HPP

#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <vector>

namespace sf::transport {

/**
 * @brief Callback invoked on each incoming BLE notification.
 *
 * Delivered on the BLE stack thread — keep the body fast (no I/O, no heavy
 * math). Enqueue raw bytes and drain on a worker thread; see Session for the
 * recommended queue pattern.
 */
using NotifyCallback = std::function<void(std::span<const uint8_t> packet)>;

/**
 * @brief Abstract BLE peripheral adapter.
 *
 * Concrete implementations:
 *   - SimpleBleAdapter  — desktop/macOS/Linux via SimpleBLE
 *   - CoreBluetoothAdapter (future) — watchOS/iOS wrapping CBPeripheral
 *
 * The protocol layer has zero knowledge of this interface; only Session and
 * application code depend on it.
 */
class IBleAdapter {
public:
    /**
     * @brief Destroy the adapter interface.
     */
    virtual ~IBleAdapter() = default;

    /**
     * @brief Scan for a peripheral with the given name and connect to it.
     *
     * Blocks until a matching device is found and connected, or until
     * @p timeout_s is exceeded.
     *
     * @param device_name  Advertised peripheral name to match.
     * @param timeout_s    Maximum scan duration in seconds.
     * @return             @c true on successful connection.
     */
    virtual bool scan_and_connect(const std::string& device_name,
                                  int timeout_s) = 0;

    /**
     * @brief Subscribe to NOTIFY on a GATT characteristic.
     *
     * @p callback is invoked from the BLE stack thread on every notification.
     *
     * @param service_uuid  128-bit service UUID string.
     * @param char_uuid     128-bit characteristic UUID string.
     * @param callback      Handler called with each notification payload.
     * @return              @c true if subscription was accepted.
     */
    virtual bool subscribe_notify(const std::string& service_uuid,
                                  const std::string& char_uuid,
                                  NotifyCallback callback) = 0;

    /**
     * @brief Write a command payload to a WRITE_WO_RSP characteristic.
     *
     * @param service_uuid  128-bit service UUID string.
     * @param char_uuid     128-bit characteristic UUID string.
     * @param payload       Raw bytes to write.
     * @return              @c true if the write was accepted.
     */
    virtual bool write_control(const std::string& service_uuid,
                               const std::string& char_uuid,
                               const std::vector<uint8_t>& payload) = 0;

    /**
     * @brief Disconnect from the current peripheral.
     */
    virtual void disconnect() = 0;

    /**
     * @brief Report whether a peripheral is currently connected.
     * @return @c true if a peripheral is currently connected.
     */
    virtual bool is_connected() = 0;

    /**
     * @brief Get the negotiated ATT MTU.
     * @return Negotiated ATT MTU in bytes, or 0 if not connected.
     */
    virtual uint16_t mtu() = 0;
};

}

#endif
