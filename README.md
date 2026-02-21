# ESP32 Rust driver for AHT10 Sensor

A Rust-based ESP32 project for reading temperature and humidity data from the AHT10 sensor via I2C communication.

- [AHT10 Sensor Datasheet](https://components101.com/sites/default/files/component_datasheet/AHT10.pdf)
- [Reference implementation in C++ @ enjoyneering/AHTxx](https://github.com/enjoyneering/AHTxx)

## Overview

This project implements a driver for the AHT10 temperature and humidity sensor on the ESP32 microcontroller. It provides both a low-level trait-based API and a higher-level `AHT10Sensor` struct for easy sensor initialization and reading.

## Features

- **AHT10 Sensor Support**: Full support for reading temperature and humidity from the AHT10 sensor
- **I2C Communication**: Uses the ESP32's I2C peripheral for sensor communication (100 kHz clock)
- **UART Output**: Transmits sensor readings via UART (115200 baud)
- **No_std Compatible**: Designed for bare-metal embedded systems without standard library dependencies
- **Optimized Build**: Release builds use LTO and single-threaded LLVM compilation for optimal performance

## Hardware Requirements

- **Microcontroller**: ESP32
- **Sensor**: AHT10 (I2C address: 0x38)
- **Connections**:
  - GPIO21: SDA (I2C Data)
  - GPIO22: SCL (I2C Clock)
  - UART0 TX: For serial output (default GPIO1)

## Project Structure

```
src/
├── lib.rs          # Library root with sensor module
├── sensor.rs       # AHT10 sensor implementation and trait definitions
└── bin/
    └── main.rs     # Example application with sensor reading loop
```

## Building

### Prerequisites

- Rust 1.88 or later
- Xtensa ESP32 toolchain

Toolchain can be auto-set after it's installed with:

```bash
source ./esp-compiler.profile
```

## Writing and flashing a program

The main application initializes the AHT10 sensor and reads temperature and humidity measurements every second via the UART console:

```rust
let mut aht10 = AHT10Sensor::new(i2c0);
aht10.initialize().unwrap();

loop {
    let measurement = aht10.read_measurement().unwrap();
    println!("Temperature: {}°F", (measurement.temperature * 1.8 + 32.0));
    println!("Humidity: {}%", measurement.humidity);
    // Sleep for 1 second between readings
}
```

Flashing is done with `cargo run` which uses `espflash`:

```bash
cargo run --release
```


### Serial Output

Connect to the ESP32 via UART to see output similar to:

```
Temperature: 77.36 °F
Humidity: 45.32 %
Temperature: 77.34 °F
Humidity: 45.31 %
```

## API

### AHT10 Trait

The `AHT10` trait defines the main sensor operations:

- `new(i2c)` - Create a new sensor instance
- `initialize()` - Initialize and calibrate the sensor
- `read_measurement()` - Read both temperature and humidity
- `read_temperature()` - Read only temperature
- `read_humidity()` - Read only humidity

### Measurement Data

```rust
pub struct AHT10Measurement {
    pub temperature: f32,  // Temperature in Celsius
    pub humidity: f32,     // Relative humidity in percentage
}
```

## Dependencies

- **esp-hal** ~1.0: Hardware abstraction layer for ESP32
- **esp-bootloader-esp-idf** 0.4.0: IDF bootloader support
- **critical-section** 1.2.0: Critical section synchronization

## Notes

- The sensor requires initialization before first use
- Measurement readings are taken with an 80ms delay
- The I2C clock frequency is set to 100 kHz for reliability
- Temperature values are returned in Celsius; the example application converts to Fahrenheit
