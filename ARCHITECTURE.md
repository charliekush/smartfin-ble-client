# Architecture

## Purpose

This repository is the shared C++ backend for all Smartfin clients. It owns packet decoding, signal processing, and the C bridge that exposes results to platform consumers.

The primary platform target is Apple Watch. Swift/CoreBluetooth owns the BLE layer on watchOS; raw notification bytes cross into this library through the C bridge. The C++ pipeline is platform-agnostic and runs the same way on desktop and watch.

This design prevents protocol and processing drift between:

- firmware in [`smartfin-fw3`](https://github.com/UCSD-E4E/smartfin-fw3)
- desktop and lab tooling
- the Apple Watch app ([`smartfin-watch`](https://github.com/charliekush/smartfin-watch))

## Ownership Boundaries

### In scope

- BLE telemetry packet framing and ensemble decoding
- ensemble IDs, protocol constants, field scaling, and unit conversion
- dequantization of fixed-point sensor fields
- AHRS orientation (Madgwick filter or on-device DMP quaternion)
- world-frame rotation and gravity subtraction
- FIR decimation, Butterworth bandpass filter, and zero-phase `filtfilt` processing
- wave metric pipeline (Welch PSD, spectral integration, moments)
- pure-C bridge API for Swift interop
- optional SimpleBLE transport adapter for desktop testing
- unit tests and CI

### Out of scope

- firmware measurement logic and embedded BLE stack behavior
- Apple Watch app UI, lifecycle, or BLE connection management
- platform-specific presentation models

## Processing Pipeline

The full pipeline runs offline after a session ends. Stages marked **[not implemented]** are planned but not yet written.

```
BLE packet (55 Hz)
    │
    ▼
Ensemble decoder                              src/protocol/
(ensemble_decoder + ensemble_types)
DecodedQuatImu → accel Q9, gyro Q7, mag Q3, Quat9 Q30
    │
    ▼
Dequantize                                    src/protocol/ensemble_decoder
accel  = raw / 512    → m/s²
gyro   = raw / 128    → deg/s
mag    = raw / 8      → µT
quat   = raw / 2³⁰   → [-1, 1]
    │
    ▼
AHRS (not yet decided)                        src/proccessing/AHRS/
  Option A: Madgwick  (accel + gyro + mag → quaternion, offline)
  Option B: DMP Quat9 (on-device quaternion already in packet, 1.1 kHz)
    │
    ▼
Rotate accel to world frame                   src/proccessing/orient_ride.cpp
accel_world = q ⊗ accel_body ⊗ q*
    │
    ▼
Subtract gravity
accel_world[z] -= 9.81
    │
    ▼
Decimate 55 Hz → 5 Hz                        src/proccessing/filter/decimator
65-tap FIR lowpass (Kaiser-window firwin, cutoff 1.0 Hz)
+ keep every 11th sample
    │
    ▼
Butterworth bandpass [0.05–0.5 Hz]            src/proccessing/filter/butterworth
filtfilt + Gustafsson IC                      src/proccessing/filter/filtfilt
    │
    ▼
[NOT IMPLEMENTED] Hann window + Welch PSD
rfft on overlapping 256-sample windows
average |X[k]|² across windows
normalize by (fs × Σ hann²)
    │
    ▼
[NOT IMPLEMENTED] Frequency-domain integration
displacement PSD = accel PSD / (2πf)⁴
zero out bins outside [0.05–0.5 Hz]
    │
    ▼
[NOT IMPLEMENTED] Spectral moments
m0 = ∫ S(f) df
m1 = ∫ f·S(f) df
m2 = ∫ f²·S(f) df
    │
    ▼
[NOT IMPLEMENTED] Wave metrics
Hs   = 4√m0
Tp   = 1 / f_peak
Tm01 = m0 / m1
Tm02 = √(m0 / m2)
    │
    ▼
Sinks                                         src/pipeline/
FileSink    → disk
LoggingSink → stdout
SampleSink  → downstream consumers / C bridge
```

## Source Layout

```
src/
  bridge/         Pure-C extern "C" API for Swift interop
  pipeline/       ISampleSink interface + FileSink, BufferSink, LoggingSink
  proccessing/
    AHRS/         Madgwick filter implementation
    filter/       Decimator, Butterworth coefficient generation, filtfilt
    math/         Matrix and linear algebra primitives
    sort/         Timsort for timestamp ordering
    processor.hpp/.cpp   Pipeline orchestration (orient → filter → …)
    proc_types.hpp/.cpp  OrientedSample, OrientedRide, RideData
    config.hpp           Filter order and cutoff constants
  protocol/       EnsembleDecoder + ensemble type definitions
  receiver/       BLE receiver (SimpleBLE sessions)
  session/        Session lifecycle
  transport/      IBleAdapter interface + SimpleBLE adapter
tests/
  butterworth/    Butterworth coefficient and property tests
  filtfilt/       filtfilt correctness tests + SciPy reference vectors
tools/
  ride_recorder.cpp   Standalone recording utility
```

## Layers

### Protocol (`src/protocol/`)

Converts raw BLE notification bytes into typed, validated structs.

Responsibilities:

- decode BLE transport packet headers
- iterate packed ensemble records
- decode known ensemble payloads into `DecodedImu`, `DecodedQuatImu`, `DecodedTemp`, `DecodedFwVersion`
- dequantize fixed-point fields and apply unit conversion

Key files:

- `ensemble_decoder.hpp/.cpp`  -  stateless decoder; call `decode()` once per BLE packet
- `ensemble_types.hpp`  -  decoded struct definitions and ensemble ID constants

### Processing (`src/proccessing/`)

Stateful offline pipeline that transforms decoded samples into processed ride output. Entry point is `Processor::process()`, which orchestrates all implemented stages in order.

**Implemented stages:**

- **AHRS**  -  Madgwick filter fuses accel, gyro, and mag into an orientation quaternion, correcting gyro drift. Whether the pipeline uses this or the on-device DMP quaternion is not yet decided.
- **Orient**  -  rotates raw IMU samples into world frame and subtracts gravity.
- **Filter**  -  65-tap FIR decimation from 55 Hz to 5 Hz, then Butterworth bandpass [0.05–0.5 Hz] applied via `filtfilt` zero-phase forward-backward filtering with Gustafsson initial-condition correction (matches SciPy output).

**Planned stages:** Welch PSD, frequency-domain integration to displacement, spectral moments, and wave metrics (Hs, Tp, Tm01, Tm02).

Supporting modules:

- `math/matrix.hpp/.tpp`  -  templated matrix used internally by `filtfilt`
- `math/lin_alg.hpp`  -  double-precision linear algebra primitives
- `sort/timsort.hpp`  -  in-place Timsort for timestamp ordering of raw samples
- `config.hpp`  -  Butterworth filter order and cutoff constants

### C Bridge (`src/bridge/`)

A thin `extern "C"` wrapper over the C++ pipeline exposing two opaque handles:

- `SF_Sink`  -  accumulates decoded samples from raw BLE notification bytes
- `SF_Proc`  -  runs the processing pipeline on a populated sink

Intended for consumption via a Swift bridging header on watchOS/iOS. CoreBluetooth hands it raw bytes; it returns `SF_OrientedSample` arrays. No BLE logic lives here.

Key files:

- `smartfin_c_api.h`  -  public C header; the only file an Apple client needs to import
- `smartfin_c_api.cpp`  -  implementation wrapping `EnsembleDecoder` and `Processor`

### Transport Adapters (`src/transport/`, `src/receiver/`)

Optional, platform-specific BLE backends for desktop and lab use. Apple clients never link this layer.

- `simpleble_adapter`  -  wraps SimpleBLE for scanning, connecting, and subscribing on macOS/Linux/Windows
- `receiver`  -  higher-level session setup on top of the adapter
- `ble_adapter.hpp`  -  abstract interface; alternative transports implement this

Enable with `SMARTFIN_ENABLE_SIMPLEBLE=1`.

### Pipeline Sinks (`src/pipeline/`)

Output targets consumed by `Processor` and tools.

- `FileSink`  -  writes decoded samples to `.sfdat` files and reads them back as `RideData`
- `BufferSink`  -  accumulates samples in memory for test harnesses
- `LoggingSink`  -  prints samples to stdout

`ISampleSink` in `sample_sink.hpp` is the interface all sinks implement.

## Language Boundary

The C++ pipeline is the implementation. The Swift boundary is the C bridge.

Pattern:

- C++ for all parsing, filtering, and processing logic
- `extern "C"` wrapper in `src/bridge/` for stable Swift interop
- Swift owns CoreBluetooth, BLE scanning, connection management, and app lifecycle
- Swift never imports C++ headers directly

## Optional SimpleBLE Support

SimpleBLE is a desktop-only transport for testing. Protocol and processing code build independently of it.

```
cmake -DSMARTFIN_ENABLE_SIMPLEBLE=1 -B build
```

## Testing

GoogleTest suite in `tests/`:

- `butterworth/`  -  filter coefficient correctness and Butterworth properties
- `filtfilt/`  -  Gustafsson initial-condition correctness and zero-phase properties, verified against reference vectors generated by `tests/filtfilt/gen_filtfilt_vectors.py` (SciPy ground truth)

CI runs on every push via GitHub Actions (`.github/workflows/tests.yml`).

## Related Repositories

- [`smartfin-fw3`](https://github.com/UCSD-E4E/smartfin-fw3)  -  firmware, embedded BLE stack, telemetry byte generation
- [`smartfin-watch`](https://github.com/charliekush/smartfin-watch)  -  Apple Watch app, CoreBluetooth, UI, app lifecycle

## Design Principles

- share processing logic, not platform assumptions
- BLE transport stays on the platform  -  never inside this library
- C bridge boundary stays narrow and stable for Swift interop
- prefer explicit binary layouts over implicit protocol assumptions
- lock protocol compatibility with golden test vectors
- make optional dependencies (SimpleBLE) truly optional
