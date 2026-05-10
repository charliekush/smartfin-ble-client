# BLE Transport + Ensemble Wire Format

## 1. Transport Header

Payload
```
Offset  Size  Field       Type      Notes
------  ----  ----------  --------  ------------------------------------------
0       1     version     uint8_t   Always 0x01
1       1     type        uint8_t   0x01 = TELEMETRY, 0x02 = STATUS
2       2     seq         uint16_t  Little-endian; monotonic per packet
4       2     payloadLen  uint16_t  Little-endian; bytes after this header
------  ----  ----------
        6     TOTAL
```

Max packet size = **236 bytes** -> max payload = **230 bytes**.

---

## 2. Ensemble Header

Payload
```
#pragma pack(push, 1)
typedef struct EnsembleHeader_ {
    unsigned int ensembleType  : 4;   // bits 0–3
    unsigned int elapsedTime_ms : 28; // bits 4–31
} EnsembleHeader_t;
```

**`sizeof(EnsembleHeader_t)` = 4 bytes** (4 + 28 = 32 bits fills one `unsigned int` exactly).

```
static_assert(sizeof(EnsembleHeader_t) == 4, "header packing mismatch");
```

**Bit layout — little-endian, LSB-first:**

```
Byte 0 = (ensembleType  & 0x0F) | ((elapsedTime_ms & 0x0F) << 4)
Byte 1 =  (elapsedTime_ms >>  4) & 0xFF
Byte 2 =  (elapsedTime_ms >> 12) & 0xFF
Byte 3 =  (elapsedTime_ms >> 20) & 0xFF
```

`elapsedTime_ms` = `(millis() − sessionStart) & 0x0FFFFFFF`
Units: **milliseconds**; 28-bit max = 268,435,455 ms ≈ **74.5 hours**.

**Decoding:**

```
uint8_t  type       = byte0 & 0x0F;
uint32_t elapsed_ms = (uint32_t)(byte0 >> 4)
                    | ((uint32_t)byte1 << 4)
                    | ((uint32_t)byte2 << 12)
                    | ((uint32_t)byte3 << 20);
float    elapsed_s  = elapsed_ms / 1000.0f;
```

---

## 3. Ensemble Payload Layouts

### 0x01 — ENS_TEMP

Payload: **3 bytes**

```
Offset  Size  Field        Encoding           Decode
------  ----  -----------  -----------------  --------------------
0       2     scaled_temp  int16_t LE, ×128   temp_°C = raw / 128
2       1     water        uint8_t            0 = dry, 1 = wet
```

---

### 0x0C — ENS_TEMP_HIGH_DATA_RATE_IMU (Ensemble12)

Payload: **18 bytes**

> **Note:** The field is named `acceleration_ms2_q14` but [`ensembles.cpp:478-480`](src/deploy/ensembles.cpp#L478-L480) encodes with `Q10_SCALAR = 1024`. Use divisor **1024** to decode acceleration.

```
Offset  Size  Field                  Encoding           Decode
------  ----  ---------------------  -----------------  ---------------------
0       2     acceleration_ms2[X]    int16_t LE, ×1024  m/s^2 = raw / 1024
2       2     acceleration_ms2[Y]    int16_t LE, ×1024
4       2     acceleration_ms2[Z]    int16_t LE, ×1024
6       2     angularVel_dps[X]      int16_t LE, ×128   dps = raw / 128
8       2     angularVel_dps[Y]      int16_t LE, ×128
10      2     angularVel_dps[Z]      int16_t LE, ×128
12      2     magIntensity_uT[X]     int16_t LE, ×8     uT = raw / 8
14      2     magIntensity_uT[Y]     int16_t LE, ×8
16      2     magIntensity_uT[Z]     int16_t LE, ×8
```

---

### 0x0D — ENS_HIGH_DATA_RATE_IMU_QUAT (Ensemble13)

Payload: **32 bytes**

Quaternion components arrive pre-quantized from the DMP via `getRawQuat9` as Q30 `int32_t` values. q0 is not transmitted — reconstruct as `sqrt(1 − q1² − q2² − q3²)`.

```
Offset  Size  Field                  Encoding           Decode
------  ----  ---------------------  -----------------  ---------------------
0       2     acceleration_ms2[X]    int16_t LE, ×512   m/s^2 = raw / 512
2       2     acceleration_ms2[Y]    int16_t LE, ×512
4       2     acceleration_ms2[Z]    int16_t LE, ×512
6       2     angularVel_dps[X]      int16_t LE, ×128   dps = raw / 128
8       2     angularVel_dps[Y]      int16_t LE, ×128
10      2     angularVel_dps[Z]      int16_t LE, ×128
12      2     magIntensity_uT[X]     int16_t LE, ×8     uT = raw / 8
14      2     magIntensity_uT[Y]     int16_t LE, ×8
16      2     magIntensity_uT[Z]     int16_t LE, ×8
18      4     quat9_q30[0]  (q1)     int32_t LE, Q30    qi = raw / 2³⁰
22      4     quat9_q30[1]  (q2)     int32_t LE, Q30
26      4     quat9_q30[2]  (q3)     int32_t LE, Q30
30      2     quat9_accuracy_q12     int16_t LE, ×4096  rad = raw / 4096
```

---

### 0x0F — ENS_TEXT (Firmware version)

Payload: **1 + nChars bytes**

```
Offset   Size     Field   Encoding  Notes
------   ------   ------  --------  ----------------------------------------
0        1        nChars  uint8_t   Character count (max 31)
1        nChars   verBuf  ASCII     Format: "FW{major}.{minor}.{build}{branch}"
```

Current version ([`src/vers.hpp`](src/vers.hpp)): `"FW3.25.0"` — 8 chars, empty branch.

---

## 4. Worked Examples

All examples use `seq = 0`. Each packet = 6-byte `PacketHeader` + 4-byte `EnsembleHeader` + payload.

---

### Example A — ENS_TEMP: 21.5 °C, in-water, t = 10 000 ms

```
scaled_temp = (int16_t)(21.5 × 128) = 2752 = 0x0AC0  ->  LE: 0xC0 0x0A

EnsembleHeader  (type=0x01, elapsedTime_ms=10000=0x2710):
  Byte 0 = (0x01 & 0xF) | ((0x0 & 0xF) << 4) = 0x01   [lower nibble of 0x2710 = 0x0]
  Byte 1 = (10000 >>  4) & 0xFF               = 0x71
  Byte 2 = (10000 >> 12) & 0xFF               = 0x02
  Byte 3 = (10000 >> 20) & 0xFF               = 0x00

payloadLen = 4 + 3 = 7
```

```
// Full packet (13 bytes):
{
  0x01, 0x01, 0x00, 0x00, 0x07, 0x00,  // PacketHeader (payloadLen=7)
  0x01, 0x71, 0x02, 0x00,              // EnsembleHeader (type=0x01, t=10000 ms)
  0xC0, 0x0A,                          // scaled_temp=2752 -> 21.5 °C
  0x01                                 // water=1
}
```

**Decoded values:**
- `temp_c     = 2752 / 128    = 21.5`
- `water      = 1`
- `elapsed_s  = 10000 / 1000 = 10.0`

---

### Example B — ENS_TEMP_HIGH_DATA_RATE_IMU: [9.807, −1.5, 0] m/s^2, [45, −30, 0] dps, [40, 5, −20] uT, t = 5 000 ms

```
accel[0] = (int16_t)(9.807  × 1024) = 10042 = 0x273A  ->  LE: 0x3A 0x27
accel[1] = (int16_t)(−1.5   × 1024) = −1536 = 0xFA00  ->  LE: 0x00 0xFA
accel[2] = 0                                            ->  LE: 0x00 0x00
gyro[0]  = (int16_t)(45.0   × 128)  =  5760 = 0x1680  ->  LE: 0x80 0x16
gyro[1]  = (int16_t)(−30.0  × 128)  = −3840 = 0xF100  ->  LE: 0x00 0xF1
gyro[2]  = 0                                            ->  LE: 0x00 0x00
mag[0]   = (int16_t)(40.0   × 8)    =   320 = 0x0140  ->  LE: 0x40 0x01
mag[1]   = (int16_t)(5.0    × 8)    =    40 = 0x0028  ->  LE: 0x28 0x00
mag[2]   = (int16_t)(−20.0  × 8)    =  −160 = 0xFF60  ->  LE: 0x60 0xFF

EnsembleHeader  (type=0x0C, elapsedTime_ms=5000=0x1388):
  Byte 0 = (0x0C & 0xF) | ((0x8 & 0xF) << 4) = 0x8C  [lower nibble of 0x1388 = 0x8]
  Byte 1 = (5000 >>  4) & 0xFF                = 0x38
  Byte 2 = (5000 >> 12) & 0xFF                = 0x01
  Byte 3 = (5000 >> 20) & 0xFF                = 0x00

payloadLen = 4 + 18 = 22
```

```
// Full packet (28 bytes):
{
  0x01, 0x01, 0x00, 0x00, 0x16, 0x00,  // PacketHeader (payloadLen=22)
  0x8C, 0x38, 0x01, 0x00,              // EnsembleHeader (type=0x0C, t=5000 ms)
  0x3A, 0x27, 0x00, 0xFA, 0x00, 0x00, // accel x,y,z (÷1024 -> m/s^2)
  0x80, 0x16, 0x00, 0xF1, 0x00, 0x00, // gyro  x,y,z (÷128  -> dps)
  0x40, 0x01, 0x28, 0x00, 0x60, 0xFF  // mag   x,y,z (÷8    -> uT)
}
```

**Decoded values:**
- `accel_ms2  = [10042/1024, −1536/1024, 0]    = [9.8066, −1.5, 0.0]`
- `gyro_dps   = [5760/128,   −3840/128,  0]    = [45.0, −30.0, 0.0]`
- `mag_uT     = [320/8, 40/8, −160/8]          = [40.0, 5.0, −20.0]`
- `elapsed_s  = 5000 / 1000                    = 5.0`

---

### Example C — ENS_HIGH_DATA_RATE_IMU_QUAT: [0, 0, 9.807] m/s^2, [0, 0, 10] dps, [30, 5, −20] uT, q=(0, 0, 0.5), acc=0.1 rad, t = 20 000 ms

```
accel[2] = (int16_t)(9.807  × 512)        =  5021 = 0x139D      ->  LE: 0x9D 0x13
gyro[2]  = (int16_t)(10.0   × 128)        =  1280 = 0x0500      ->  LE: 0x00 0x05
mag[0]   = (int16_t)(30.0   × 8)          =   240 = 0x00F0      ->  LE: 0xF0 0x00
mag[1]   = (int16_t)(5.0    × 8)          =    40 = 0x0028      ->  LE: 0x28 0x00
mag[2]   = (int16_t)(−20.0  × 8)          =  −160 = 0xFF60      ->  LE: 0x60 0xFF
q3_Q30   = (int32_t)(0.5 × 2³⁰)          = 536870912 = 0x20000000 ->  LE: 0x00 0x00 0x00 0x20
acc_q12  = (int16_t)(0.1 × 4096)          =   409 = 0x0199      ->  LE: 0x99 0x01

EnsembleHeader  (type=0x0D, elapsedTime_ms=20000=0x4E20):
  Byte 0 = (0x0D & 0xF) | ((0x0 & 0xF) << 4) = 0x0D  [lower nibble of 0x4E20 = 0x0]
  Byte 1 = (20000 >>  4) & 0xFF               = 0xE2
  Byte 2 = (20000 >> 12) & 0xFF               = 0x04
  Byte 3 = (20000 >> 20) & 0xFF               = 0x00

payloadLen = 4 + 32 = 36
```

```
// Full packet (42 bytes):
{
  0x01, 0x01, 0x00, 0x00, 0x24, 0x00,  // PacketHeader (payloadLen=36)
  0x0D, 0xE2, 0x04, 0x00,              // EnsembleHeader (type=0x0D, t=20000 ms)
  0x00, 0x00, 0x00, 0x00, 0x9D, 0x13, // accel x,y,z (÷512  -> m/s^2)
  0x00, 0x00, 0x00, 0x00, 0x00, 0x05, // gyro  x,y,z (÷128  -> dps)
  0xF0, 0x00, 0x28, 0x00, 0x60, 0xFF, // mag   x,y,z (÷8    -> uT)
  0x00, 0x00, 0x00, 0x00,             // q1 = 0.0 (Q30)
  0x00, 0x00, 0x00, 0x00,             // q2 = 0.0 (Q30)
  0x00, 0x00, 0x00, 0x20,             // q3 = 0.5 (Q30)
  0x99, 0x01                          // heading accuracy ≈ 0.1 rad (Q12)
}
```

**Decoded values:**
- `accel_ms2    = [0, 0, 5021/512]               = [0.0, 0.0, 9.8066]`
- `gyro_dps     = [0, 0, 1280/128]               = [0.0, 0.0, 10.0]`
- `mag_uT       = [240/8, 40/8, −160/8]          = [30.0, 5.0, −20.0]`
- `q1,q2,q3     = [0, 0, 536870912/1073741824]   = [0.0, 0.0, 0.5]`
- `q0           = sqrt(1 − 0 − 0 − 0.25)        = 0.8660`
- `accuracy_rad = 409 / 4096                     = 0.09985`
- `elapsed_s    = 20000 / 1000                   = 20.0`

---

### Example D — ENS_TEXT: FW3.25.0, t = 1 000 ms

```
"FW3.25.0" ASCII = {0x46, 0x57, 0x33, 0x2E, 0x32, 0x35, 0x2E, 0x30}
nChars = 8

EnsembleHeader  (type=0x0F, elapsedTime_ms=1000=0x3E8):
  Byte 0 = (0x0F & 0xF) | ((0x8 & 0xF) << 4) = 0x8F  [lower nibble of 0x3E8 = 0x8]
  Byte 1 = (1000 >>  4) & 0xFF                = 0x3E
  Byte 2 = (1000 >> 12) & 0xFF                = 0x00
  Byte 3 = (1000 >> 20) & 0xFF                = 0x00

payloadLen = 4 + 1 + 8 = 13
```

```
// Full packet (19 bytes):
{
  0x01, 0x01, 0x00, 0x00, 0x0D, 0x00,              // PacketHeader (payloadLen=13)
  0x8F, 0x3E, 0x00, 0x00,                           // EnsembleHeader (type=0x0F, t=1000 ms)
  0x08,                                             // nChars = 8
  0x46, 0x57, 0x33, 0x2E, 0x32, 0x35, 0x2E, 0x30  // "FW3.25.0"
}
```

**Decoded values:**
- `version   = std::string(verBuf, nChars) = "FW3.25.0"`
- `elapsed_s = 1000 / 1000                = 1.0`

---

## 5. Packet Assembly

- One `PacketHeader` wraps the entire notification; each ensemble inside carries its own 4-byte `EnsembleHeader`.
- Structure: `[6-byte PacketHeader][4+N ens₁][4+M ens₂]...`
- High-rate records (0x0D) are enqueued as `HighRateImuRecord` = `EnsembleHeader_t` (4 B) + `Ensemble13_data_t` (32 B) = **36 bytes**; up to **6 records** fit per notification (6 × 36 = 216 ≤ 230).
- Low-rate ensembles (0x01, 0x0F) go through `enqueueLowRateEnsemble` and may be batched with any ensemble type in the same packet.
