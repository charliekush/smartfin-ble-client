/**
 * @file simpleble_adapter.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief IBleAdapter implementation backed by the SimpleBLE library.
 * @date 2026-04-30
 * 
 */

#ifndef SIMPLEBLE_ADAPTER_HPP
#define SIMPLEBLE_ADAPTER_HPP

#include "ble_adapter.hpp"

#include <optional>
#include <simpleble/SimpleBLE.h>

namespace sf::transport {

/**
 * @brief IBleAdapter backed by the SimpleBLE library.
 *
 * Targets desktop platforms (macOS, Linux, Windows). This translation unit
 * is excluded from the build when @c SMARTFIN_ENABLE_SIMPLEBLE=OFF, allowing
 * Apple Watch / iOS targets to link only the protocol and session layers.
 */
class SimpleBleAdapter final : public IBleAdapter {
public:
    /**
     * @brief Construct an empty adapter wrapper.
     */
    SimpleBleAdapter() = default;

    /**
     * @brief Destroy the adapter and disconnect if a device is still connected.
     */
    ~SimpleBleAdapter() override;

    /**
     * @brief Scan for a Smartfin peripheral and connect to it.
     * @param device_name Advertised peripheral name to match.
     * @param timeout_s Maximum scan duration in seconds.
     * @return @c true if a matching device was found and connected.
     */
    bool scan_and_connect(const std::string& device_name,
                          int timeout_s) override;

    /**
     * @brief Subscribe to notifications on a characteristic.
     * @param service_uuid GATT service UUID string.
     * @param char_uuid GATT characteristic UUID string.
     * @param callback Callback invoked for each incoming payload.
     * @return @c true if subscription succeeded.
     */
    bool subscribe_notify(const std::string& service_uuid,
                          const std::string& char_uuid,
                          NotifyCallback callback) override;

    /**
     * @brief Write a control payload without waiting for a response.
     * @param service_uuid GATT service UUID string.
     * @param char_uuid GATT characteristic UUID string.
     * @param payload Raw bytes to send.
     * @return @c true if the write was accepted by SimpleBLE.
     */
    bool write_control(const std::string& service_uuid,
                       const std::string& char_uuid,
                       const std::vector<uint8_t>& payload) override;

    /**
     * @brief Disconnect from the currently connected peripheral.
     */
    void     disconnect()  override;

    /**
     * @brief Report whether a peripheral is currently connected.
     * @return @c true if a peripheral is connected.
     */
    bool     is_connected() override;

    /**
     * @brief Get the negotiated ATT MTU.
     * @return ATT MTU in bytes, or 0 when no peripheral is connected.
     */
    uint16_t mtu()         override;

private:
    /**
     * @brief Connected peripheral handle.
     */
    std::optional<SimpleBLE::Peripheral> peripheral_;
};

}

#endif
