# Swift / Xcode Integration

Apple clients should build and link only the protocol, pipeline, processing, and C bridge layers. Do not link the SimpleBLE desktop transport into an iOS or watchOS target.

Swift owns CoreBluetooth. For each BLE notification, Swift passes the raw notification bytes into the C bridge. The C++ backend decodes packets, runs processing, and returns C-compatible result structs.

## Build The Bridge

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

## Minimal Swift Usage

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
