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
- AHRS orientation — Madgwick filter (offline) or on-device DMP quaternion (not yet decided)
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

## Architecture

```
┌─────────────────────────────────────────────────┐
│              Platform (Swift / desktop)          │
│                                                 │
│  CoreBluetooth (watchOS/iOS)                    │
│       or SimpleBLE (desktop testing)            │
└──────────────┬──────────────────────────────────┘
               │ raw BLE notification bytes
               ▼
┌─────────────────────────────────────────────────┐
│               C Bridge  (src/bridge/)            │
│                                                 │
│  sf_sink_push_packet()  →  sf_proc_run()        │
│  pure-C extern "C" API for Swift interop        │
└──────────────┬──────────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────────┐
│            Protocol Layer  (src/protocol/)       │
│                                                 │
│  EnsembleDecoder → DecodedImu / DecodedQuatImu  │
│                    DecodedTemp / DecodedFwVersion│
└──────────────┬──────────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────────┐
│          Processing Pipeline  (src/proccessing/) │
│                                                 │
│  Madgwick AHRS  →  orient_ride                  │
│  Butterworth + filtfilt  →  filtered accel      │
│  Processor::process()  →  ProcessedRide         │
└──────────────┬──────────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────────┐
│          Pipeline Sinks  (src/pipeline/)         │
│                                                 │
│  FileSink  /  BufferSink  /  LoggingSink        │
└─────────────────────────────────────────────────┘
```

## Layers

### Protocol (`src/protocol/`)

Converts raw BLE notification bytes into structured data. Owns ensemble framing, header parsing, payload decoding, and field scaling. This is the canonical bytes-to-meaning layer — no consumer should reimplement this logic independently.

### Processing (`src/proccessing/`)

Stateful offline pipeline that transforms decoded samples into processed ride data:

1. **AHRS** — Madgwick filter fuses accelerometer, gyroscope, and magnetometer into an orientation quaternion, correcting gyroscope drift.
2. **Orient** — rotates raw IMU samples into world frame using the AHRS quaternion output.
3. **Filter** — FIR decimation followed by Butterworth IIR coefficients + `filtfilt` zero-phase filtering applied to world-frame acceleration axes.

### C Bridge (`src/bridge/`)

A thin `extern "C"` wrapper over the C++ pipeline, exposing opaque `SF_Sink` and `SF_Proc` handles. Intended for consumption via a Swift bridging header on watchOS/iOS. Usage:

```c
SF_Sink* sink = sf_sink_create();
SF_Proc* proc = sf_proc_create();

// For each CoreBluetooth notification:
sf_sink_push_packet(sink, bytes, length);

// After session ends:
size_t n = 0;
SF_OrientedSample* result = sf_proc_run(proc, sink, &n);
// ... consume result[0..n-1] ...
sf_oriented_free(result);

sf_proc_destroy(proc);
sf_sink_destroy(sink);
```

## Swift / Xcode Integration

Apple clients should build and link only the protocol, pipeline, processing, and C bridge layers. Do not link the SimpleBLE desktop transport into an iOS or watchOS target.

Build the bridge target without SimpleBLE:

```bash
cmake -B build -DSMARTFIN_ENABLE_BRIDGE=ON -DSMARTFIN_ENABLE_SIMPLEBLE=OFF
cmake --build build --target smartfin_bridge
```

Then add the generated static libraries to the Xcode target and make `src/bridge/smartfin_c_api.h` visible to Swift through the app's bridging header:

```objc
#include "bridge/smartfin_c_api.h"
```

When linking manually in Xcode, include the bridge library and its C++ dependencies: `libsmartfin_bridge.a`, `libsmartfin_proc.a`, `libsmartfin_filter.a`, `libsmartfin_ahrs.a`, `libsmartfin_math.a`, `libsmartfin_pipeline.a`, and `libsmartfin_protocol.a`. Also link the C++ standard library (`libc++`).

For real iOS/watchOS distribution, build the static library for the target Apple SDK and architecture you need, or package those builds as an `.xcframework`. A default desktop CMake build is useful for local validation, but it is not automatically a watchOS/iOS artifact.

Minimal Swift usage:

```swift
final class SmartfinProcessor {
    private let sink = sf_sink_create()
    private let proc = sf_proc_create()

    deinit {
        sf_proc_destroy(proc)
        sf_sink_destroy(sink)
    }

    func pushNotification(_ data: Data) {
        data.withUnsafeBytes { rawBuffer in
            guard let base = rawBuffer.bindMemory(to: UInt8.self).baseAddress else {
                return
            }
            _ = sf_sink_push_packet(sink, base, rawBuffer.count)
        }
    }

    func finishRide() -> [SF_OrientedSample] {
        var count = 0
        guard let ptr = sf_proc_run(proc, sink, &count) else {
            return []
        }
        defer { sf_oriented_free(ptr) }

        let buffer = UnsafeBufferPointer(start: ptr, count: count)
        return Array(buffer)
    }

    func reset() {
        sf_sink_clear(sink)
    }
}
```

`sf_proc_run` returns heap memory owned by the C bridge. Swift callers must release it with `sf_oriented_free` after copying or consuming the returned samples.

### Transport Adapters (`src/transport/`, `src/receiver/`)

Optional, platform-specific BLE backends. The `SimpleBLE` adapter is for desktop and lab testing only. Apple clients use Swift/CoreBluetooth and never link this layer.

Enable with:

```
SMARTFIN_ENABLE_SIMPLEBLE=1
```

### Pipeline Sinks (`src/pipeline/`)

Output targets for the processing pipeline. `FileSink` writes processed data to `.sfdat` files; `BufferSink` accumulates in memory; `LoggingSink` prints samples.

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
- [`smartfin-watch`](https://github.com/charliekush/smartfin-watch) — Apple Watch app, CoreBluetooth, UI, app lifecycle

## Design Principles

- share processing logic, not platform assumptions
- BLE transport stays on the platform (Swift or SimpleBLE) — never in this library
- C bridge boundary stays narrow and stable for Swift interop
- prefer explicit binary layouts over implicit assumptions
- lock protocol compatibility with golden test vectors
- make optional dependencies (SimpleBLE) truly optional
