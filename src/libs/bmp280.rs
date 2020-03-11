use std::thread;
use std::time::Duration;

use i2cdev::core::*;
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};

/***************
mod sensors {
    use std::error::Error;

    pub struct BMP280Sample {
        pub temperatur: f32,
        pub pressure: f32,
        pub pressure_nn: f32,
        //pub Dev: LinuxI2CDevice,
    }

    pub trait Thermometer {
        type Error: Error;
        fn get_temperature(&mut self) -> Result<f32, Self::Error>;
        fn get_pressure(&mut self) -> Result<f32, Self::Error>;
        fn get_pressure_nn(&mut self) -> Result<f32, Self::Error>;
    }

    pub mod bmp280_temp_sensor {
        use super::*;
        use byteorder::{ByteOrder, LittleEndian};
        use i2cdev::core::I2CDevice;
        use std::error::Error;
        use std::thread;
        use std::time::Duration;

        pub const BMP280_I2C_ADDR_PRIM: u8 = 0x76;
        pub const BMP280_CHIP_ID: u8 = 0x60;

        ///! name Register Address 
        pub const BMP280_CHIP_ID_ADDR: u8 = 0xD0;
        const BMP280_RESET_ADDR: u8 = 0xE0;
        const BMP280_STATUS_ADDR:u8 = 0xF3;
        const BMP280_TEMP_PRESS_CALIB_DATA_ADDR: u8 = 0x88;
        const BMP280_HUMIDITY_CALIB_DATA_ADDR: u8 = 0xE1;
        const BMP280_PWR_CTRL_ADDR: u8 = 0xF4;
        const BMP280_CTRL_HUM_ADDR: u8 = 0xF2;
        const BMP280_CTRL_MEAS_ADDR: u8 = 0xF4;
        const BMP280_CONFIG_ADDR: u8 = 0xF5;
        const BMP280_DATA_ADDR: u8 = 0xF7;

        const BMP280_SOFT_RESET_CMD: u8 = 0xB6;

        const BMP280_TEMP_PRESS_CALIB_DATA_LEN: u8 = 24;

        const ALTITUDE: i32 = 500;
        const SLEEP_TIME: u64 = 40;

        pub struct BMP280TempSensor<T: I2CDevice + Sized> {
            i2cdev: T,
        }

        impl<T> BMP280TempSensor<T> where T: I2CDevice + Sized, {
            pub fn new(mut i2cdev: T) -> Result<BMP280TempSensor<T>, T::Error> {
                Ok(BMP280TempSensor { i2cdev })
            }
            
            fn get_data(&mut self) -> Result<BMP280Sample, T::Error> {   
                let mut comp_data: [u8; 6] = [0; 6];
                
                // read humidity
                self.i2cdev.smbus_write_byte(BMP280_DATA_ADDR);
                thread::sleep(Duration::from_millis(SLEEP_TIME));
        
                self.i2cdev.read(&mut comp_data);
                thread::sleep(Duration::from_millis(10));
        
                // Convert pressure and temperature data to 19-bits
                let adc_p = (comp_data[0] << 12) + (comp_data[1] << 4) + (comp_data[2] >> 4);
                let adc_t = (comp_data[3] << 12) + (comp_data[4] << 4) + (comp_data[5] >> 4);
        
                // temperature offset calculations
                let temp1 = adc_t / 16384.0 - (T[0]/1024.0)*T[1];
                let temp3 = adc_t / 131072.0 - (T[0]/8192.0);
                let temp2 = temp3 * temp3 * T[2];
        
                // pressure offset calculations
                let press1 = ((temp1 + temp2) / 2.0) - 64000.0;
                let press2 = press1 * press1 * P[5] / 32768.0;
                press2 = press2 + press1 * P[4] * 2.0;
                press2 = (press2 / 4.0) + (P[3] * 65536.0);
                press1 = P[2] * press1 * press1 / 524288.0 + ( P[1] * press1) / 524288.0;
                press1 = (1.0 + press1 / 32768.0) * (P[0]);
                let press3 = 1048576.0 - adc_p;
                if (press1 != 0.0) {
                    press3 = (press3 - press2 / 4096.0) * 6250.0 / press1;
                    press1 = P[8]) * press3 * press3 / 2147483648.0;
                    press2 = press3 * P[7] / 32768.0;
                    pres = (press3 + (press1 + press2 + (P[6])) / 16.0) / 100;
                } else { 
                    pres = 0.0; 
                }
                
                Ok(BMP280Sample {
                    temperatur: (temp1 + temp2) / 5120.0;
                    pressure: pres;
                    pressure_nn = pres / pow(1 - ALTITUDE/44330.0, 5.255);
                })
            }
        }

        impl<T> Tempsensor for BMP280TempSensor<T> where T: I2CDevice + Sized, {
            fn process(&mut self) {}
            
            fn soft_reset() {
                let rslt: bool;
                
                self.Dev.smbus_write_byte_data(BMP280_RESET_ADDR, BMP280_SOFT_RESET_CMD);
                thread::sleep(Duration::from_millis(SLEEP_TIME));
            }
        
            fn get_calib_data() {
                
                let reg_addr: u8 = BME280_TEMP_PRESS_CALIB_DATA_ADDR;
                // Array to store calibration data 
                let mut calib_data: [u8: BMP280_TEMP_PRESS_CALIB_DATA_LEN] = [0u8; BMP280_TEMP_PRESS_CALIB_DATA_LEN];
        
                // Read the calibration data from the sensor 
                self.Dev.write(&[reg_addr])?;
                self.Dev.read(&mut calib_data)?;
        
                parse_temp_calib_data(calib_data);
            }
        
            fn parse_temp_calib_data(&mut reg_data: u8) {
                T[0] = LittleEndian::read_i16(&reg_data[0], &reg_data[1]);
                T[1] = LittleEndian::read_i16(&reg_data[2], &reg_data[3]);
                
                if(T[1] > 32767) {
                    T[1] -= 65536; 
                }
                
                T[2] = LittleEndian::read_i16(&reg_data[4], &reg_data[5]);
                if(T[2] > 32767) { 
                    T[2] -= 65536; 
                }
                
                P[0] = LittleEndian::read_i16(&reg_data[6], &reg_data[7]);
                for (int i = 0; i < 8; i++) {
                    P[i+1] = reg_data[2*i+9]*256 + reg_data[2*i+8];
                    if(P[i+1] > 32767) { 
                        P[i+1] -= 65536; 
                    }
                }
            }
        

        }
    }
}
*/

pub const BMP280_I2C_ADDR_PRIM: u8 = 0x76;
pub const BMP280_CHIP_ID: u8 = 0x60;

///! name Register Address 
pub const BMP280_CHIP_ID_ADDR: u8 = 0xD0;
const BMP280_RESET_ADDR: u8 = 0xE0;
const BMP280_STATUS_ADDR:u8 = 0xF3;
const BMP280_TEMP_PRESS_CALIB_DATA_ADDR: u8 = 0x88;
const BMP280_HUMIDITY_CALIB_DATA_ADDR: u8 = 0xE1;
const BMP280_PWR_CTRL_ADDR: u8 = 0xF4;
const BMP280_CTRL_HUM_ADDR: u8 = 0xF2;
const BMP280_CTRL_MEAS_ADDR: u8 = 0xF4;
const BMP280_CONFIG_ADDR: u8 = 0xF5;
const BMP280_DATA_ADDR: u8 = 0xF7;
const BMP280_SOFT_RESET_CMD: u8 = 0xB6;
const BMP280_TEMP_PRESS_CALIB_DATA_LEN: usize = 24;
const ALTITUDE: f32 = 500.0;
const SLEEP_TIME: u64 = 40;

pub struct BMP280 {
	pub temperature: f32,
    pub pressure: f32,
    pub pressure_nn: f32,
    T: [f32; 3],
    P: [f32; 9],
    Dev: LinuxI2CDevice,
}

pub trait BMPSensor {
    fn new(dev_name: &'static str) -> Self;
    // fn getTemp(&mut self) -> f32;
    // fn getPres(&mut self) -> f32;
    // fn getPresNN(&mut self) -> f32;
    fn process(&mut self);
}

impl BMP280 {
    pub fn Setup(&mut self) -> Result<(), LinuxI2CError> {
        // soft resetpressure_nn
        self.soft_reset();
        println!("Soft Reset done!");

        let mut chip_id: u8 = self.Dev.smbus_read_byte_data(BMP280_CHIP_ID_ADDR)?;
        // self.Dev.write(&[BMP280_CHIP_ID_ADDR])?;
        // self.Dev.read(&mut chip_id)?;

        if chip_id == BMP280_CHIP_ID {
            self.get_calib_data();
            self.Dev.smbus_write_byte_data(BMP280_CTRL_MEAS_ADDR, 0x27)?;
            self.Dev.smbus_write_byte_data(BMP280_CONFIG_ADDR, 0xA0);
            thread::sleep(Duration::from_millis(1000));
        }
        
        Ok(())
    }

    pub fn soft_reset(&self) {
        let rslt: bool;
        
        self.Dev.smbus_write_byte_data(BMP280_RESET_ADDR, BMP280_SOFT_RESET_CMD);
        thread::sleep(Duration::from_millis(SLEEP_TIME));
    }

    fn get_calib_data(&mut self) -> Result<(), LinuxI2CError> {
        let reg_addr: u8 = BMP280_TEMP_PRESS_CALIB_DATA_ADDR;
        // Array to store calibration data 
        let mut calib_data: [u8; BMP280_TEMP_PRESS_CALIB_DATA_LEN] = [0u8; BMP280_TEMP_PRESS_CALIB_DATA_LEN];

        // Read the calibration data from the sensor 
        self.Dev.write(&[reg_addr])?;
        self.Dev.read(&mut calib_data)?;

        self.parse_temp_calib_data(&calib_data);
        Ok(())
    }

    fn parse_temp_calib_data(&mut self, reg_data: &[u8]) {
        let mut t: [u16; 3] = [0; 3];
        let mut p: [u16; 9] = [0; 9];

        t[0] = ((reg_data[0] as u16) << 8) | reg_data[1] as u16;
        t[1] = ((reg_data[2] as u16) << 8) | reg_data[3] as u16;
        if t[1] > 32767 { 
            t[1] -= 65536; 
        }

        t[2] = ((reg_data[4] as u16) << 8) | reg_data[5] as u16;
        if t[2] > 32767 { 
            t[2] -= 65536; 
        }
        
        self.T[0] = t[0] as f32;
        self.T[1] = t[1] as f32;
        self.T[2] = t[2] as f32;

        p[0] = ((reg_data[6] as u16) << 8) | reg_data[7] as u16;
        self.P[0] = p[0] as f32;

        for i in 0..8 {
            p[i+1] = reg_data[2*i+9] * 256 + reg_data[2*i+8];            
            if p[i+1] > 32767 { 
                p[i+1] -= 65536; 
            }
            self.P[i+1] = p[i+1] as f32
        }
    }

    pub fn Process(&mut self) {   
        let mut comp_data: [u8; 6] = [0; 6];
        
        // read humidity
        self.Dev.smbus_write_byte(BMP280_DATA_ADDR);
        thread::sleep(Duration::from_millis(SLEEP_TIME));

        self.Dev.read(&mut comp_data);
        thread::sleep(Duration::from_millis(10));

        // Convert pressure and temperature data to 19-bits
        let adc_p: f32 = ((comp_data[0] << 12) + (comp_data[1] << 4) + (comp_data[2] >> 4)) as f32;
        let adc_t: f32 = ((comp_data[3] << 12) + (comp_data[4] << 4) + (comp_data[5] >> 4)) as f32;

        // temperature offset calculations
        let temp1 = adc_t / 16384.0 - (self.T[0] / 1024.0) * self.T[1];
        let temp3 = adc_t / 131072.0 - (self.T[0] / 8192.0);
        let temp2 = temp3 * temp3 * self.T[2];
        self.temperature = (temp1 + temp2) / 5120.0;

        // pressure offset calculations
        let mut press1 = ((temp1 + temp2) / 2.0) - 64000.0;
        let mut press2 = press1 * press1 * self.P[5] / 32768.0;
        press2 = press2 + press1 * self.P[4] * 2.0;
        press2 = (press2 / 4.0) + (self.P[3] * 65536.0);
        press1 = self.P[2] * press1 * press1 / 524288.0 + (self.P[1] * press1) / 524288.0;
        press1 = (1.0 + press1 / 32768.0) * (self.P[0]);
        let press3 = 1048576.0 - adc_p;
        if press1 != 0.0 {
            press3 = (press3 - press2 / 4096.0) * 6250.0 / press1;
            press1 = self.P[8] * press3 * press3 / 2147483648.0;
            press2 = press3 * self.P[7] / 32768.0;
            self.pressure = (press3 + (press1 + press2 + (self.P[6])) / 16.0) / 100.0;
        } else { 
            self.pressure = 0.0; 
        }
        let p: f32 = 1 - ALTITUDE / 44330.0;
        self.pressure_nn = self.pressure / p.powf(5.255);
    }
}

impl BMPSensor for BMP280 {
    fn new(dev_name: &'static str) -> BMP280 {
        BMP280 { 
            temperature: 0.0,
            pressure: 0.0,
            pressure_nn: 0.0,
            T: [0, 3],
            P: [0, 9],
            Dev: LinuxI2CDevice::new(dev_name, BMP280_I2C_ADDR_PRIM.into()).unwrap()
        }
    }

    fn process(&mut self) {
        self.Process()
    }
}
