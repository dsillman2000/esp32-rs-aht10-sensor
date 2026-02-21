use esp_hal::{
    Blocking,
    i2c::master::{Error as I2cError, I2c},
    time::Instant,
};

use core::fmt::{self, Error as FmtError, Write};

pub trait AHT10<'d> {
    fn new(i2c: I2c<'d, Blocking>) -> Self;
    fn get_calibration(&mut self) -> Result<bool, InitializeError>;
    fn soft_reset(&mut self) -> Result<bool, InitializeError>;
    fn set_normal_mode(&mut self) -> Result<(), InitializeError>;
    fn set_initialization_register(&mut self, mode: u8) -> Result<(), InitializeError>;
    fn initialize(&mut self) -> Result<bool, InitializeError>;
    fn read_measurement(&mut self) -> Result<AHT10Measurement, InitializeError>;
    fn read_temperature(&mut self) -> Result<f32, InitializeError>;
    fn read_humidity(&mut self) -> Result<f32, InitializeError>;
}

const AHT10_POWER_ON_DELAY_MS: u32 = 40;
const AHT10_CMD_DELAY_MS: u32 = 350;
const AHT10_MEASUREMENT_DELAY_MS: u32 = 80;
const AHT10_ERROR: u8 = 0xFF;
const AHT10_INIT_CMD: u8 = 0xE1;
const AHT10_START_MEASUREMENT_CMD: u8 = 0xAC;
const AHT10_NORMAL_CMD: u8 = 0xA8;
const AHT10_SOFT_RESET_CMD: u8 = 0xBA;
const AHT10_INIT_NORMAL_MODE: u8 = 0x00;
const AHT10_INIT_CYCLE_MODE: u8 = 0x20;
const AHT10_INIT_CMD_MODE: u8 = 0x40;
const AHT10_DATA_NOP: u8 = 0x00;
const AHT10_DATA_MEASUREMENT_CMD: u8 = 0x33;
const AHT10_INIT_CAL_ENABLE: u8 = 0x08;
const AHT10_FORCE_READ_DATA: bool = true;
const AHT10_USE_READ_DATA: bool = false;
const AHT10_DEVICE_ADDRESS: u8 = 0x38;
const AHT10_STATUS_REG: u8 = 0x71;

pub struct AHT10Sensor<'d> {
    i2c: I2c<'d, Blocking>,
    raw_data: [u8; 6],
}

fn _delay(duration_ms: u64) {
    let mut delay = Instant::now();
    while delay.elapsed().as_millis() < duration_ms as u64 {}
}

pub enum InitializeError {
    Fmt(FmtError),
    I2c(I2cError),
}

impl From<FmtError> for InitializeError {
    fn from(err: FmtError) -> Self {
        InitializeError::Fmt(err)
    }
}

impl From<I2cError> for InitializeError {
    fn from(err: I2cError) -> Self {
        InitializeError::I2c(err)
    }
}

impl fmt::Debug for InitializeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            InitializeError::Fmt(err) => write!(f, "Format Error: {}", err),
            InitializeError::I2c(err) => write!(f, "I2C Error: {}", err),
        }
    }
}

pub struct AHT10Measurement {
    pub temperature: f32,
    pub humidity: f32,
}

fn decode_measurement(measurement_data: &[u8; 6]) -> AHT10Measurement {
    // Decode the raw measurement data into temperature and humidity values.
    let raw_humidity = ((measurement_data[1] as u32) << 12)
        | ((measurement_data[2] as u32) << 4)
        | ((measurement_data[3] as u32) >> 4);
    let raw_temperature = (((measurement_data[3] as u32) & 0x0F) << 16)
        | ((measurement_data[4] as u32) << 8)
        | (measurement_data[5] as u32);

    let humidity = (raw_humidity as f32) * 100.0 / (1 << 20) as f32;
    let temperature = (raw_temperature as f32) * 200.0 / (1 << 20) as f32 - 50.0;

    AHT10Measurement {
        temperature,
        humidity,
    }
}

impl<'d> AHT10<'d> for AHT10Sensor<'d> {
    fn new(i2c_peripheral: I2c<'d, Blocking>) -> Self {
        Self {
            i2c: i2c_peripheral,
            raw_data: [AHT10_ERROR, 0, 0, 0, 0, 0],
        }
    }

    fn set_initialization_register(&mut self, mode: u8) -> Result<(), InitializeError> {
        // Set the initialization register based on the desired mode.
        _delay(AHT10_CMD_DELAY_MS as u64);
        self.i2c.write(
            AHT10_DEVICE_ADDRESS,
            &[AHT10_INIT_CMD, mode, AHT10_DATA_NOP],
        )?;
        Ok(())
    }

    fn set_normal_mode(&mut self) -> Result<(), InitializeError> {
        // Set the sensor to Normal Mode.
        self.set_initialization_register(AHT10_INIT_CAL_ENABLE | AHT10_INIT_NORMAL_MODE)
    }

    fn get_calibration(&mut self) -> Result<bool, InitializeError> {
        // Read the calibration bit from the sensor.
        _delay(AHT10_CMD_DELAY_MS as u64);
        let mut status_data = [0u8; 1];
        self.i2c
            .write_read(AHT10_DEVICE_ADDRESS, &[AHT10_STATUS_REG], &mut status_data)?;
        let calibration_bit = status_data[0] & 0x08 != 0;
        Ok(calibration_bit)
    }

    fn soft_reset(&mut self) -> Result<bool, InitializeError> {
        // Perform a soft reset of the sensor.
        self.i2c
            .write(AHT10_DEVICE_ADDRESS, &[AHT10_SOFT_RESET_CMD])?;
        _delay(AHT10_POWER_ON_DELAY_MS as u64);
        let calibration = self.get_calibration()?;
        Ok(calibration)
    }

    fn initialize(&mut self) -> Result<bool, InitializeError> {
        // Initialize the sensor in Normal Mode.
        _delay(AHT10_POWER_ON_DELAY_MS as u64);
        Ok(self.soft_reset()?)
    }

    fn read_measurement(&mut self) -> Result<AHT10Measurement, InitializeError> {
        // Trigger a measurement and read the raw data from the sensor.
        self.i2c.write_read(
            AHT10_DEVICE_ADDRESS,
            &[
                AHT10_START_MEASUREMENT_CMD,
                AHT10_DATA_MEASUREMENT_CMD,
                AHT10_DATA_NOP,
            ],
            &mut self.raw_data,
        )?;
        _delay(AHT10_MEASUREMENT_DELAY_MS as u64);
        let measurement = decode_measurement(&self.raw_data);
        Ok(measurement)
    }

    fn read_temperature(&mut self) -> Result<f32, InitializeError> {
        // Implementation details
        let measurement = self.read_measurement()?;
        return Ok(measurement.temperature);
    }

    fn read_humidity(&mut self) -> Result<f32, InitializeError> {
        // Implementation details
        let measurement = self.read_measurement()?;
        return Ok(measurement.humidity);
    }
}
