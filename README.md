# smartfin-ble-client

Cross-platform C++ backend for Smartfin telemetry processing. The BLE layer lives on each platform (Swift/CoreBluetooth on Apple, SimpleBLE on desktop); this library handles everything from raw packet bytes to processed, world-frame sensor data.

## Overview

This repository is the shared backend for all Smartfin clients. It decodes BLE telemetry packets, runs the signal processing pipeline (AHRS orientation, Butterworth filtering, zero-phase filtfilt), and exposes results through a pure-C API that any platform can consume.

The primary target is Apple Watch. Swift owns the CoreBluetooth BLE layer and hands raw notification bytes into the C bridge. The C++ pipeline runs entirely on-device without any platform-specific code.

This separation prevents protocol drift between:

- firmware in [`smartfin-fw3`](https://github.com/UCSD-E4E/smartfin-fw3)
- desktop and lab tooling
- the Apple Watch app ([`smartfin-watch`](https://github.com/charliekush/smartfin-watch))

## What This Repository Owns

- BLE telemetry packet framing, ensemble decoding, and dequantization
- ensemble IDs, protocol constants, field scaling, and unit conversion
- AHRS orientation with Madgwick filter (offline) or on-device DMP quaternion (not yet decided)
- world-frame rotation and gravity subtraction
- Butterworth bandpass [0.05–0.5 Hz] filter coefficient generation
- `filtfilt` zero-phase forward-backward filtering with Gustafsson IC (matches SciPy)
- wave metric pipeline: decimation and wave-band filtering implemented; Welch PSD, spectral integration, moments, Hs/Tp/Tm01/Tm02 *(planned)*
- pure-C bridge API for Swift interop (`src/bridge/`)
- optional host-side SimpleBLE transport adapter for desktop testing
- GoogleTest unit test suite with SciPy-generated reference vectors
- GitHub Actions CI

## What It Does Not Own

- firmware measurement logic or embedded BLE stack behavior
- Apple Watch app UI, lifecycle, or BLE connection management
- platform-specific presentation models

## Documentation

- [Architecture](ARCHITECTURE.md) — module boundaries, processing stages, transport ownership, and design rationale.
- [Swift / Xcode Integration](docs/SWIFT_INTEGRATION.md) — C bridge usage, Xcode linking notes, and a minimal Swift example.

## Building

```bash
cmake -B build
cmake --build build
```

Run tests:

```bash
cd build && ctest
```

## Testing

GoogleTest suite covering:

- Butterworth filter coefficient correctness and filter properties
- `filtfilt` Gustafsson initial-condition method
- `filtfilt` zero-phase properties verified against SciPy reference vectors (`tests/filtfilt/gen_filtfilt_vectors.py`)

CI runs on every push via GitHub Actions.

## Related Repositories

- [`smartfin-fw3`](https://github.com/UCSD-E4E/smartfin-fw3) — firmware, embedded BLE stack, telemetry byte generation


## Design Principles

- share processing logic, not platform assumptions
- BLE transport stays on the platform (Swift or SimpleBLE) — never in this library
- C bridge boundary stays narrow and stable for Swift interop
- prefer explicit binary layouts over implicit assumptions
- lock protocol compatibility with golden test vectors
- make optional dependencies (SimpleBLE) truly optional
