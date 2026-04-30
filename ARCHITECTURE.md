# Architecture

## Purpose

This repository is the source of truth for the Smartfin host-side telemetry protocol.

Its role is to ensure that every consumer interprets Smartfin telemetry bytes the same way while allowing each platform to use its own BLE transport implementation.

This architecture is intended to prevent protocol drift between:

- firmware in [`smartfin-fw3`](https://github.com/UCSD-E4E/smartfin-fw3)
- desktop and lab tooling
- future client applications such as the Apple Watch app

## Ownership Boundaries

### In scope

This repository owns:

- BLE telemetry packet framing
- ensemble IDs and protocol constants
- ensemble header parsing
- binary decoding of ensemble payloads
- field scaling and unit conversion rules
- protocol documentation
- golden test vectors
- conformance tests
- optional host-side BLE transport adapters for testing

### Out of scope

This repository does not own:

- firmware measurement logic
- embedded BLE stack behavior
- Apple Watch app UI or lifecycle code
- platform-specific presentation models

## Layered Design

The system is split into three layers:

1. Protocol
2. Transport adapters
3. Client applications and tools

### 1. Protocol layer

The protocol layer is shared across all consumers. It is responsible for converting raw telemetry bytes into consistent, validated, structured data.

Responsibilities:

- decode BLE transport packet headers
- iterate packed ensemble records
- decode known ensemble payloads
- apply field scaling and validation

This is the canonical bytes-to-meaning layer. Any consumer that needs to understand Smartfin telemetry should rely on this layer rather than reimplementing protocol parsing independently.

### 2. Transport adapter layer

Transport adapters are optional and platform-specific. Their job is to move raw bytes between the device and the protocol decoder without owning protocol interpretation.

Responsibilities:

- scan for devices
- connect
- subscribe to telemetry notifications
- write control messages
- deliver raw bytes to the protocol decoder

Examples:

- a `SimpleBLE` adapter for desktop and lab testing
- a future native Apple transport built with `CoreBluetooth`

Transport code should remain isolated from decoding logic so that protocol behavior stays portable and testable.

### 3. Application layer

Applications and tools consume decoded telemetry and decide how to use it.

Examples:

- logging tools
- protocol test harnesses
- client applications

This layer owns presentation, workflow, and product behavior. It should not redefine protocol semantics.

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
