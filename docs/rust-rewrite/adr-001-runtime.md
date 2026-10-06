# ADR 001: bounded Rust core with ESP-IDF SDK boundary

Decision date: 2026-10-05. Host implementation accepted for this development slice; exact ESP32-S3 archive and SDK image link verified; hardware feasibility remains open.

Use one synchronous Rust owner. `flight-core`, `flight-protocol`, `flight-config`, and the board owner are `no_std`. Only host replay and the simulator ABI use `std`. Startup may allocate in ESP-IDF or the host handle registry; the core tick does not own a heap.

The first board candidate is ESP32-S3 with MPU6050 on 400 kHz I²C. Actual supplier variant, markings, pins, power, and motor gate circuit have not been verified. The compiled profile defaults to unwired and PWM disabled. It cannot serve as a flight configuration.

Use ESP-IDF **v5.4.2** through a small explicit C SDK boundary and the Espressif Rust **v1.88.0.0** toolchain candidate. Host Rust is **1.92.0**. Build scripts reject another SDK/tag/compiler. The target Rust archive has its own locked dependency graph and aborting panic handler. This SDK boundary was selected instead of broad generated HAL bindings to keep driver lifetime, polling, gate writes and callback handoffs reviewable. This is a deviation from the original `esp-idf-hal` candidate; the exact ESP32-S3 build passes in the recorded evidence; timing and physical behavior remain acceptance gates.

ESP-IDF provides the required LEDC, I²C, UART, UDP, NVS and ESP-NOW APIs. `esp-hal` remains an alternative if measured scheduling/driver latency or the independent gate strategy is inadequate. No async runtime is needed for the 1 kHz owner. Networking and driver allocation costs remain target measurements.

The MPU6050 uses ±4g, ±2000°/s, DLPF=1 and divider=0. This gives the documented 1 kHz gyro sample path. It intentionally differs from the legacy unspecified `DLPF_MAX` choice; sensor/filter parity therefore remains open. Register identity/configuration readback is mandatory. Two seconds of stationary samples estimates gyro bias; this is a heuristic, not evidence of airframe calibration.

MAVLink is a narrow common-dialect codec with frozen ID/CRC/length inventory and source SHA-256, rather than the complete generated `mavlink` crate. The choice limits parser storage and supported command surface. Unknown messages and incompatible masks are denied. Its wire contract must remain checked against primary dialect definitions and independent clients. Signing uses SHA-256 and four replay streams; a trustworthy per-boot timestamp floor and actual client provisioning remain mandatory external gates. IDs alone never authenticate Wi-Fi.

Reversal criteria: target cannot link the archive, acquisition exceeds the declared budget, SDK calls block the flight path, signing cannot interoperate with the required client, or the external gate cannot demonstrate stopped output during stall/reset. Record a replacement ADR before changing runtime or controller tuning.

Primary references: [Rust Xtensa release](https://github.com/esp-rs/rust-build/releases/tag/v1.88.0.0), [IDF LEDC](https://docs.espressif.com/projects/esp-idf/en/v5.4.2/esp32s3/api-reference/peripherals/ledc.html), [IDF I²C](https://docs.espressif.com/projects/esp-idf/en/v5.4.2/esp32s3/api-reference/peripherals/i2c.html), [IDF ESP-NOW](https://docs.espressif.com/projects/esp-idf/en/v5.4.2/esp32s3/api-reference/network/esp_now.html), [MAVLink serialization](https://mavlink.io/en/guide/serialization.html), [MAVLink signing](https://mavlink.io/en/guide/message_signing.html).
