/**
 * @file simpleble_adapter.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief IBleAdapter implementation backed by the SimpleBLE library.
 * @date 2026-04-30
 */

#ifndef SF_TRANSPORT_SIMPLEBLE_ADAPTER_HPP
#define SF_TRANSPORT_SIMPLEBLE_ADAPTER_HPP

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
    SimpleBleAdapter() = default;

    /// Disconnects on destruction if still connected.
    ~SimpleBleAdapter() override;

    bool scan_and_connect(const std::string& device_name,
                          int timeout_s) override;

    bool subscribe_notify(const std::string& service_uuid,
                          const std::string& char_uuid,
                          NotifyCallback callback) override;

    bool write_control(const std::string& service_uuid,
                       const std::string& char_uuid,
                       const std::vector<uint8_t>& payload) override;

    void     disconnect()  override;
    bool     is_connected() override;
    uint16_t mtu()         override;

private:
    /// Connected peripheral, empty when not connected.
    std::optional<SimpleBLE::Peripheral> peripheral_;
};

} // namespace sf::transport

#endif // SF_TRANSPORT_SIMPLEBLE_ADAPTER_HPP
