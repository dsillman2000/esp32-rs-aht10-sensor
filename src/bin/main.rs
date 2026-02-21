#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::fmt::Write;
use esp_hal::{
    clock::CpuClock,
    gpio::OutputConfig,
    i2c::master::{Config as I2cConfig, I2c, SoftwareTimeout},
    main,
    time::{Instant, Rate},
    uart::{Config as UartConfig, Uart},
};
use my_esp_project::sensor::{AHT10, AHT10Sensor};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[main]
fn main() -> ! {
    // generator version: 1.2.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let i2c_config = I2cConfig::default().with_frequency(Rate::from_khz(100));
    // .with_software_timeout(SoftwareTimeout::Transaction(Duration::from_millis(1000)));
    let uart_config = UartConfig::default().with_baudrate(115200);
    let output_config = OutputConfig::default();

    let mut i2c0 = I2c::new(peripherals.I2C0, i2c_config)
        .unwrap()
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22);
    let mut uart0 = Uart::new(peripherals.UART0, uart_config).unwrap();
    let mut aht10 = AHT10Sensor::new(i2c0);
    aht10.initialize().unwrap();

    // Application loop
    loop {
        let delay = Instant::now();
        let measurement = aht10.read_measurement().unwrap();
        writeln!(
            uart0,
            "Temperature: {:?} °F",
            (measurement.temperature * 1.8f32 + 32f32)
        )
        .unwrap();
        writeln!(uart0, "Humidity: {:?} %", measurement.humidity).unwrap();
        while delay.elapsed().as_millis() < 1000 {
            // Perform sensor readings or other tasks here
        }
    }
}
