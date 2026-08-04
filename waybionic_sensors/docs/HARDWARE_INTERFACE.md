# IMU Hardware Interface

Everything the software side needs from electrical before a real IMU driver can
be written, plus what is deliberately left unimplemented until those answers
arrive.

**Status: awaiting answers from electrical.** No value below is confirmed. The
package ships a mock source and an unimplemented driver stub rather than a
guessed serial protocol.

## Questions for electrical

### Sensor

| # | Question | Answer |
|---|----------|--------|
| 1 | Exact sensor model and manufacturer part number? | _unanswered_ |
| 2 | Link to the datasheet used for the selection? | _unanswered_ |
| 3 | Does the device output a fused quaternion, or only raw accelerometer and gyroscope? | _unanswered_ |
| 4 | Is there a magnetometer, and will it be usable near the motors? | _unanswered_ |
| 5 | Is on-chip filtering enabled, and at what cutoff? | _unanswered_ |

### Transport

| # | Question | Answer |
|---|----------|--------|
| 6 | How does the IMU reach the ground-station computer: direct USB, USB-serial bridge, microcontroller relay, or I2C/SPI on a board? | _unanswered_ |
| 7 | If a microcontroller sits in between, what is its packet format: framing bytes, field order, endianness, checksum? | _unanswered_ |
| 8 | Baud rate or bus clock, and is it configurable? | _unanswered_ |
| 9 | Device path or enumeration rule on Linux, and should a udev rule be provided? | _unanswered_ |
| 10 | Does the device timestamp its own samples, or must the host stamp on receipt? | _unanswered_ |

### Data format

| # | Question | Answer |
|---|----------|--------|
| 11 | Units as transmitted: g or m/s^2, deg/s or rad/s, raw counts with a scale factor? | _unanswered_ |
| 12 | Axis convention and mounting orientation relative to `base_link`? | _unanswered_ |
| 13 | Sample rate, and is it fixed or configurable? | _unanswered_ |
| 14 | Per-axis noise density or RMS noise for covariance values? | _unanswered_ |
| 15 | Is bias/scale calibration applied on-device, or expected from the host? | _unanswered_ |

### Integration

| # | Question | Answer |
|---|----------|--------|
| 16 | Where is the IMU physically mounted, and what is its transform from `base_link`? | _unanswered_ |
| 17 | Is there a status or fault line worth surfacing in `/diagnostics`? | _unanswered_ |
| 18 | Expected behaviour on cable disconnect: silence, error frames, or reconnect? | _unanswered_ |

## Known unknowns

Until the answers arrive, these stay open and are not guessed anywhere in code:

- Wire protocol and framing. `hardware_reader.py` defines an interface only.
- Real covariance values. The mock uses placeholder standard deviations exposed
  as parameters (`angular_velocity_stddev`, `linear_acceleration_stddev`).
- Whether a fused orientation will ever be available. Until it is, the raw topic
  marks orientation unavailable.
- The static transform from `base_link` to `imu_link`. The demo TF is a
  visualisation aid with a placeholder 0.1 m offset, not a mounting claim.

## Adding the driver later

The node reads samples through
`waybionic_sensors.hardware_reader.ImuHardwareReader`. Adding hardware means
implementing that interface in a new module and constructing it instead of
`UnconfiguredImuReader`:

```python
class MyImuReader(ImuHardwareReader):
    def open(self): ...
    def read(self, stamp_ns) -> Optional[ImuReading]: ...
    def close(self): ...
    def describe(self) -> str: ...
```

The driver owns transport and parsing, and converts to the REP-103 units of
`ImuReading`. Message construction, covariance, diagnostics, and TF need no
changes. Parser tests should be added at that point using recorded packets from
the real device.

Live mode already works end to end with the stub: the node publishes no samples
and `imu.heartbeat` reports STALE, which is the correct depiction of a missing
sensor.
