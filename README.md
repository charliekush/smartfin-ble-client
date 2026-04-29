# smartfin-ble-client

Shared Smartfin protocol definitions and decoding libraries for BLE telemetry, with optional host-side transport adapters for testing and client development.

## Overview

This repository is the source of truth for the Smartfin host-side telemetry protocol. Its job is to give every consumer the same interpretation of Smartfin telemetry bytes while leaving BLE transport details to the platform that uses them.

That separation helps prevent protocol drift between:

- firmware in [`smartfin-fw3`](https://github.com/UCSD-E4E/smartfin-fw3)
- desktop and lab tooling
- future client applications such as the Apple Watch app

## What This Repository Owns

- BLE telemetry packet framing
- ensemble IDs and protocol constants
- ensemble header parsing
- binary decoding of ensemble payloads
- field scaling and unit conversion rules
- protocol documentation
- golden test vectors
- conformance tests
- optional host-side BLE transport adapters for testing

## What It Does Not Own

- firmware measurement logic
- embedded BLE stack behavior
- Apple Watch app UI or lifecycle code
- platform-specific presentation models

## Architecture

The system is split into three layers:

1. Protocol
2. Transport adapters
3. Client applications and tools

### Protocol layer

The protocol layer is shared across all consumers and is responsible for:

- decoding BLE transport packet headers
- iterating packed ensemble records
- decoding known ensemble payloads
- applying field scaling and validation

This is the canonical bytes-to-meaning layer.

### Transport adapter layer

Transport adapters are optional and platform-specific. They are responsible for:

- scanning
- connecting
- subscribing to telemetry notifications
- writing control messages
- delivering raw bytes to the protocol decoder

Examples include a `SimpleBLE` adapter for desktop testing and a future native Apple transport built with `CoreBluetooth`.

### Application layer

Applications and tools consume decoded telemetry and decide how to use it. Examples include logging tools, protocol test harnesses, and client applications.

## Optional SimpleBLE Support

If this repository includes a `SimpleBLE` backend, it should be treated as:

- optional
- host-side only
- one transport implementation among several possible adapters

Recommended behavior:

- protocol code builds independently of `SimpleBLE`
- `SimpleBLE` support is enabled only when explicitly requested
- Apple clients can reuse the shared decoder without compiling the desktop transport backend

Example build flag:

`SMARTFIN_ENABLE_SIMPLEBLE=1`

## Apple Platform Guidance

For Apple Watch and other Apple clients:

- keep BLE transport logic in Swift using `CoreBluetooth`
- pass raw notification payloads into the shared Smartfin decoder
- use a thin C wrapper around the shared decoder if Swift interop is needed

This keeps protocol logic shared without forcing Apple apps to depend on a desktop-oriented BLE backend.

## Language Boundary

The protocol implementation may use C++ internally, but any boundary shared with Swift should expose a small C-compatible API.

Recommended pattern:

- C++ for parser implementation
- a thin `extern "C"` wrapper for stable interop
- Swift owning Apple BLE transport integration

## Related Repositories

### `smartfin-fw3`

Owns firmware-side generation of telemetry bytes and embedded behavior.

### `smartfin-watch`

Owns Apple Watch BLE connection logic, app lifecycle, and UI.

### `smartfin-ble-client`

Owns the Smartfin wire format, decoding logic, protocol documentation, test vectors, and optional host-side test transports.

## Design Principles

- share protocol logic, not platform assumptions
- keep transport adapters isolated from decoding logic
- prefer explicit binary layouts over implicit assumptions
- use golden test vectors to lock compatibility
- make optional dependencies truly optional
- keep Swift interop boundaries narrow and stable
