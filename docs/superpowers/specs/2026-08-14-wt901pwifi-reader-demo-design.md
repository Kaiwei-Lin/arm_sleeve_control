# WT901PWIFI Real-Time Reader Demo Design

## Goal

Create an independently runnable Python demo that reads and decodes real-time WT901PWIFI data through Type-C serial, UDP, or TCP without changing the project's existing IMU770 integration.

## Source Material

- WT901PWIFI product specification: <https://wit-motion.yuque.com/wumwnr/docs/gdm0fgbic0bi97ry>
- WT901PWIFI protocol: <https://wit-motion.yuque.com/wumwnr/docs/cutvgcs7up421vpu>
- WT901PWIFI quick start: <https://wit-motion.yuque.com/wumwnr/docs/kwknrb1y68xcf5e4>

The product specification identifies a Type-C UART interface with a default rate of 9600 bps and 2.4 GHz WiFi supporting TCP and UDP. The device defaults to AP mode and UDP. Its real-time record is a fixed 54-byte binary frame beginning with `57 54 35 35` (`WT55`) and ending with `0D 0A`.

## Scope

The deliverable consists of:

- `tools/read_wt901pwifi.py`: the standalone reader and protocol implementation.
- `tests/test_wt901pwifi_demo.py`: hardware-free protocol and transport tests.
- A short README section documenting setup and example commands.

The demo will not modify `sleeve_arm.sources`, `configs/sensors.yaml`, the existing IMU770 parser, synchronization code, or robot-control behavior.

## Command-Line Interface

The script uses transport subcommands:

```text
python tools/read_wt901pwifi.py serial --port COM5
python tools/read_wt901pwifi.py udp --host 0.0.0.0 --port 1399
python tools/read_wt901pwifi.py tcp-server --host 0.0.0.0 --port 1399
python tools/read_wt901pwifi.py tcp-client --host 192.168.4.1 --port 9250
```

Serial mode defaults to 9600 bps, eight data bits, no parity, and one stop bit. UDP and TCP server modes bind to the supplied host and port. TCP client mode connects to the supplied sensor endpoint for configurations where the sensor accepts a connection. Network sockets and accepted TCP connections use a finite timeout so Ctrl+C and shutdown remain responsive.

The default output is one human-readable line per decoded frame. `--json` emits one JSON object per frame for piping into other programs. Both output modes expose all decoded fields.

## Architecture

The implementation stays in one executable file because it is a standalone demonstration, but its internals have three explicit boundaries:

1. `WT901PFrame` is an immutable dataclass containing the decoded device ID, device time, acceleration, angular velocity, magnetic field, Euler angles, temperature, battery voltage, RSSI, and firmware version.
2. `WT901PStreamParser.feed(data)` owns a byte buffer, locates the `WT55` header, validates the fixed length and `CRLF` terminator, resynchronizes after malformed input, and returns zero or more decoded frames.
3. Transport runners read byte chunks from serial, UDP, a TCP listener, or a TCP client and pass every chunk to the same parser. They do not interpret protocol fields.

This split keeps protocol decoding independent from I/O and lets tests exercise real parsing behavior without hardware.

## Protocol Decoding

All two-byte numeric fields are little-endian. Motion and environmental fields are signed 16-bit values unless the protocol defines them as unsigned. The following conversions are applied:

- Acceleration: `raw / 32768 * 16` g.
- Angular velocity: `raw / 32768 * 2000` degrees per second.
- Magnetic field: `raw * 100 / 1024` microtesla.
- Roll, pitch, and yaw: `raw / 32768 * 180` degrees.
- Temperature: `raw / 100` degrees Celsius.
- Battery voltage: `raw / 100` volts.
- RSSI: signed integer in dBm.
- Firmware version: unsigned 16-bit integer.

Bytes 4 through 11 contain the eight ASCII digits of the device ID. Bytes 12 through 19 contain year, month, day, hour, minute, second, and little-endian milliseconds. Bytes 20 through 51 contain the sixteen two-byte values in the order documented by the vendor. Bytes 52 and 53 must be `0D 0A`.

The vendor's 54-byte streaming record does not define a checksum. Structural validation therefore consists of the exact header, exact length, ASCII device-ID validation, plausible date/time fields, and the `CRLF` terminator. Invalid candidates are discarded one byte at a time until the next header can be found, allowing recovery from noise, partial data, and concatenated frames.

## WiFi Roles and Defaults

The sensor defaults to AP mode and UDP. In the vendor protocol, the default remote computer address is `192.168.4.2`, the default remote TCP/UDP port is `1399`, and the sensor local port is `9250`.

Accordingly:

- `udp` is a local UDP receiver, normally bound to `0.0.0.0:1399` while the computer is connected to the sensor hotspot.
- `tcp-server` is the primary TCP mode and waits for a sensor configured to connect to the computer's IP and port.
- `tcp-client` is also included for sensor configurations that expose a listening endpoint, but no connection to `192.168.4.1:9250` is assumed unless the user explicitly supplies it.

The demo only receives measurement data. It does not write device registers, change WiFi credentials, alter output rate, calibrate sensors, or persist settings.

## Error Handling and Shutdown

- Missing `pyserial` produces an installation hint only when serial mode is selected.
- Serial-open, bind, listen, accept, and connect failures report the selected endpoint and exit with a nonzero code.
- Socket or serial timeouts are normal polling events, not fatal errors.
- A disconnected TCP peer closes that connection. TCP server mode returns to accepting another peer; TCP client mode exits with a clear message.
- Malformed protocol candidates increment a parser counter and do not stop acquisition.
- Ctrl+C closes the active serial port, socket, accepted connection, and listener before exiting successfully.

## Testing

Tests will be written before production code and will use synthetic vendor-format frames. They will cover:

- Decoding positive and negative values with the documented scale factors.
- Device ID, device time, temperature, battery, RSSI, and version fields.
- A frame split across multiple `feed` calls.
- Multiple frames in a single chunk.
- Leading garbage and recovery after a malformed terminator.
- UDP reception on a loopback ephemeral port.
- TCP server reception on a loopback ephemeral port.
- TCP client reception from a loopback test server.
- CLI argument defaults and JSON serialization.

Hardware validation remains a user-run step because no WT901PWIFI is available to the automated test environment. Successful hardware validation means each transport can continuously print changing measurements, Ctrl+C exits promptly, and a stationary device reports approximately 1 g total acceleration with near-zero angular velocity.

## Documentation

The README addition will summarize the verified device parameters, explain the default AP/UDP topology, list serial/UDP/TCP commands, describe Windows firewall implications for inbound UDP/TCP, and state that the existing IMU770 integration is unrelated to this standalone WT901PWIFI demo.
