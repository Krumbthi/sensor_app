extern crate i2cdev;
extern crate byteorder;
extern crate core;

use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};
use i2cdev::core::I2CDevice;
use byteorder::{LittleEndian, BigEndian, WriteBytesExt, ReadBytesExt};
use std::io::Cursor;

const DEFAULT_I2C_ADDRESS: u16 = 0x77;
const DEFAULT_I2C_PATH: &'static str = "/dev/i2c-1";

/// Wrapper type for results
pub type Result<T> = std::result::Result<T, Error>;

///
type Endiness = BigEndian;

/// Errors that all functions could return. Errors will either be from the i2cdev library or the
/// byteorder library.
#[derive(Debug)]
pub enum Error {
    I2cError(LinuxI2CError),
    IoError(std::io::Error),
    Other(()),
}

impl From<LinuxI2CError> for Error {
    fn from(f: LinuxI2CError) -> Self {
        Error::I2cError(f)
    }
}

impl From<std::io::Error> for Error {
    fn from(f: std::io::Error) -> Self {
        Error::IoError(f)
    }
}

impl From<()> for Error {
    fn from(f: ()) -> Self {
        Error::Other(f)
    }
}

/// All of the registers for the BMP280
enum Register {
    DigT1,
    DigT2,
    DigT3,

    DigP1,
    DigP2,
    DigP3,
    DigP4,
    DigP5,
    DigP6,
    DigP7,
    DigP8,
    DigP9,

    ChipId,
    Version,
    SoftReset,

    /// R calibration stored in 0xE1-0xF0
    Cal26,

    Control,
    Config,
    PressureData,
    TemperatureData,
}

impl<'a> std::convert::From<&'a Register> for u8 {
    fn from(frm: &'a Register) -> u8 {
        use Register::*;
        match *frm {
            DigT1 => 0x88,
            DigT2 => 0x8A,
            DigT3 => 0x8C,

            DigP1 => 0x8E,
            DigP2 => 0x90,
            DigP3 => 0x92,
            DigP4 => 0x94,
            DigP5 => 0x96,
            DigP6 => 0x98,
            DigP7 => 0x9A,
            DigP8 => 0x9C,
            DigP9 => 0x9E,

            ChipId => 0xD0,
            Version => 0xD1,
            SoftReset => 0xE0,

            Cal26 => 0xE1,

            Control => 0xF4,
            Config => 0xF5,
            PressureData => 0xF7,
            TemperatureData => 0xFA,
        }
    }
}

/// Calibration data for the BMP280. There is no need to create this struct manually, it will
/// automatically be created.
struct Calibration {
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,

    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,

    dig_h1: u8,
    dig_h2: i16,
    dig_h3: u8,
    dig_h4: i16,
    dig_h5: i16,
    dig_h6: i8,
}

impl core::default::Default for Calibration {
    fn default() -> Self {
        unsafe { core::mem::zeroed() }
    }
}

/// A single BMP280 sensor
pub struct Bmp280 {
    sensor_id: i32,
    fine: i32,
    calibration: Calibration,
    i2c_device: LinuxI2CDevice,
    ground_pressure: f32,
}

/// A builder for Bmp280 sensors.
///
/// ```ignore
/// let mut sensor = Bmp280Builder::new()
///     .address(0x20)
///     .path("/dev/i2c-1".to_string())
///     .build().ok("Failed to build device");
///
/// let altitude = sensor.read_altitude();
///
/// // Minimal example
/// let mut sensor = Bmp280Builder::new().build().ok("Failed to build device");
/// let altitude = sensor.read_altitude();
/// ```
pub struct Bmp280Builder {
    i2c_address: u16,
    i2c_path: String,
    ground_pressure: f32,
}

impl Bmp280Builder {
    pub fn new() -> Self {
        Bmp280Builder {
            i2c_address: DEFAULT_I2C_ADDRESS,
            i2c_path: DEFAULT_I2C_PATH.to_string(),
            ground_pressure: 0.,
        }
    }

    /// Set the address of the I2C device for the sensor. There is a default value for this, so you
    /// do not need to specify it explicitly.
    pub fn address(&mut self, address: u16) -> &mut Self {
        self.i2c_address = address;
        self
    }

    /// Set the path of the I2C device for the sensor.  There is a default value for this, so you
    /// do not need to specify it explicitly.
    pub fn path(&mut self, path: String) -> &mut Self {
        self.i2c_path = path;
        self
    }

    /// Set the ground pressure for the sensor. If you do not specify this, the altitude will be
    /// zeroed when you call .build().
    pub fn ground_pressure(&mut self, pressure: f32) -> &mut Self {
        self.ground_pressure = pressure;
        self
    }

    /// Attempt to build a Bmp280 sensor from this builder.
    pub fn build(&self) -> Result<Bmp280> {
        let dev = LinuxI2CDevice::new(&self.i2c_path, self.i2c_address).unwrap();

        let mut sensor = Bmp280 {
            i2c_device: dev,
            sensor_id: 0,
            calibration: Calibration::default(),
            fine: 0,
            ground_pressure: self.ground_pressure,
        };

        sensor.begin().unwrap();

        if self.ground_pressure != 0. {
            sensor.zero().unwrap();
        }

        Ok(sensor)
    }
}

impl Bmp280 {
    fn write8(&mut self, reg: &Register, value: u8) -> Result<()> {
        self.i2c_device.write(&[reg.into()]).unwrap();
        self.i2c_device.write(&[value]).unwrap();
        Ok(())
    }

    /// Will set the relative pressure for ground level readings for .read_altitude().
    pub fn zero(&mut self) -> Result<()> {
        self.ground_pressure = self.read_pressure().unwrap();

        Ok(())
    }

    fn read8(&mut self, reg: &Register) -> Result<u8> {
        let mut buf = [0u8; 1];

        self.i2c_device.write(&[reg.into()]).unwrap();
        self.i2c_device.read(&mut buf).unwrap();

        let mut curs = Cursor::new(buf);

        let val = curs.read_u8().unwrap();

        Ok(val)
    }

    fn write16(&mut self, reg: &Register, value: u16) -> Result<()> {
        let mut buf = vec![0u8, 0u8];
        buf.write_u16::<Endiness>(value).unwrap();

        self.i2c_device.write(&[reg.into()]).unwrap();
        self.i2c_device.write(&buf).unwrap();

        Ok(())
    }

    fn read16(&mut self, reg: &Register) -> Result<u16> {
        let mut buf = [0u8; 2];

        self.i2c_device.write(&[reg.into()]).unwrap();
        self.i2c_device.read(&mut buf).unwrap();

        let mut curs = Cursor::new(buf);

        let val = curs.read_u16::<Endiness>().unwrap();

        Ok(val)
    }

    fn read16s(&mut self, reg: &Register) -> Result<i16> {
        let mut buf = [0u8; 2];

        self.i2c_device.write(&[reg.into()]).unwrap();
        self.i2c_device.read(&mut buf).unwrap();

        let mut curs = Cursor::new(buf);

        let val = curs.read_i16::<Endiness>().unwrap();

        Ok(val)
    }

    fn read16le(&mut self, reg: &Register) -> Result<u16> {
        let mut buf = [0u8; 2];

        self.i2c_device.write(&[reg.into()]).unwrap();
        self.i2c_device.read(&mut buf).unwrap();

        let mut curs = Cursor::new(buf);

        let val = curs.read_u16::<LittleEndian>().unwrap();

        Ok(val)
    }

    fn read16les(&mut self, reg: &Register) -> Result<i16> {
        let mut buf = [0u8; 2];

        self.i2c_device.write(&[reg.into()]).unwrap();
        self.i2c_device.read(&mut buf).unwrap();

        let mut curs = Cursor::new(buf);

        let val = curs.read_i16::<LittleEndian>().unwrap();

        Ok(val)
    }

    fn read24(&mut self, reg: &Register) -> Result<u32> {
        let mut buf = [0u8; 3];

        self.i2c_device.write(&[reg.into()]).unwrap();
        self.i2c_device.read(&mut buf).unwrap();

        let mut curs = Cursor::new(buf);

        let val = curs.read_uint::<Endiness>(3).unwrap();

        Ok(val as u32)
    }

    fn read_coefficients(&mut self) -> Result<()> {
        self.calibration.dig_t1 = self.read16le(&Register::DigT1).unwrap();
        self.calibration.dig_t2 = self.read16les(&Register::DigT2).unwrap();
        self.calibration.dig_t3 = self.read16les(&Register::DigT3).unwrap();

        self.calibration.dig_p1 = self.read16le(&Register::DigP1).unwrap();
        self.calibration.dig_p2 = self.read16les(&Register::DigP2).unwrap();
        self.calibration.dig_p3 = self.read16les(&Register::DigP3).unwrap();
        self.calibration.dig_p4 = self.read16les(&Register::DigP4).unwrap();
        self.calibration.dig_p5 = self.read16les(&Register::DigP5).unwrap();
        self.calibration.dig_p6 = self.read16les(&Register::DigP6).unwrap();
        self.calibration.dig_p7 = self.read16les(&Register::DigP7).unwrap();
        self.calibration.dig_p8 = self.read16les(&Register::DigP8).unwrap();
        self.calibration.dig_p9 = self.read16les(&Register::DigP9).unwrap();

        Ok(())
    }

    fn begin(&mut self) -> Result<()> {
        if (self.read8(&Register::ChipId).unwrap()) != 0x60 {
            return Err(Error::Other(()));
        }

        self.read_coefficients().unwrap();
        self.write8(&Register::Control, 0x3F).unwrap();

        Ok(())
    }

    pub fn read_temperature(&mut self) -> Result<f32> {
        let mut adc_t = self.read24(&Register::TemperatureData).unwrap() as i32;
        adc_t >>= 4;

        let t1 = self.calibration.dig_t1 as i32;
        let t2 = self.calibration.dig_t2 as i32;
        let t3 = self.calibration.dig_t3 as i32;

        let var1 = ((((adc_t >> 3) - (t1 << 1))) * t2) >> 11;
        let var2 = (((((adc_t >> 4) - t1) * ((adc_t >> 4) - t1)) >> 12) * t3) >> 14;

        self.fine = var1 + var2;

        let t = ((self.fine * 5 + 128) >> 8) as f32;
        Ok(t / 100.)
    }

    pub fn read_pressure(&mut self) -> Result<f32> {
        // This is done to initialize the self.fine value.
        self.read_temperature().unwrap();

        let adc_p = (self.read24(&Register::PressureData).unwrap() as i32) >> 4;

        let p1 = self.calibration.dig_p1 as i64;
        let p2 = self.calibration.dig_p2 as i64;
        let p3 = self.calibration.dig_p3 as i64;
        let p4 = self.calibration.dig_p4 as i64;
        let p5 = self.calibration.dig_p5 as i64;
        let p6 = self.calibration.dig_p6 as i64;
        let p7 = self.calibration.dig_p7 as i64;
        let p8 = self.calibration.dig_p8 as i64;
        let p9 = self.calibration.dig_p9 as i64;

        let var1 = (self.fine as i64) - 128000;

        let var2 = var1 * var1 * p6;
        let var2 = var2 + ((var1 * p5) << 17);
        let var2 = var2 + (p4 << 35);

        let var1 = ((var1 * var1 * p3) >> 8) + ((var1 * p2) << 12);
        let var1 = ((((1i64) << 47) + var1)) * (p1) >> 33;

        if var1 == 0 {
            return Err(Error::Other(()));
        }


        let p: i64 = 1048576 - adc_p as i64;
        let p = (((p << 31) - var2) * 3125) / var1;

        let var1 = (p9 * (p >> 13) * (p >> 13)) >> 25;
        let var2 = (p8 * p) >> 19;

        let p = ((p + var1 + var2) >> 8) + (p7 << 4);

        Ok(p as f32 / 256.)
    }

    /// Reads the altitude from the sensor relative to the given sea level pressure.
    pub fn read_altitude_relative_to(&mut self, sea_level_hpa: f32) -> Result<f32> {
        let pressure = self.read_pressure().unwrap() as f32 / 100.;

        let altitude = 44330. * (1. - (pressure / sea_level_hpa).powf(0.1903));
        Ok(altitude)
    }

    /// Reads the altitude from the sensor relative to the zeroed altitude set by .zero(),
    /// Bmp280Builder.ground_pressure(), or Bmp280Builder.build() if you do not set a ground
    /// pressure.
    pub fn read_altitude(&mut self) -> Result<f32> {
        let pressure = self.ground_pressure;

        self.read_altitude_relative_to(pressure)
    }
}

