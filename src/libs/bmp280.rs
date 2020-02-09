use std::thread;
use std::time::Duration;

use i2cdev::core::*;
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};

const BMP280_I2C_ADDR_PRIM: u8 = 0x76;
const BMP280_CHIP_ID: u8 = 0x60;

///! name Register Address 
const BMP280_CHIP_ID_ADDR: u8 = 0xD0;
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


pub struct BMP280Sensor {
    pub temperatur: f32,
    pub pressure: f32,
    pub pressure_nn: f32,
    pub Dev: LinuxI2CDevice,
}

impl BMP280Sensor {
    pub fn Setup(&mut self) {
        // soft resetpressure_nn
        self.soft_reset();
        println!("Soft Reset done!");

        let chip_id: u8 = self.Dev.smbus_read_byte_data(BMP280_CHIP_ID_ADDR);

        if (chip_id == BMP280_CHIP_ID) {
            rslt = this->GetCalibData();
            config[0] = 0xF4;
            config[1] = 0x27;
            rslt = IFace->WriteData(BME280_CTRL_MEAS_ADDR, config, ARRAY_SIZE(config));
            config[0] = 0xF5;
            config[1] = 0xA0;
            rslt = IFace->WriteData(BME280_CONFIG_ADDR, config, ARRAY_SIZE(config));
            usleep(1000);
        }
    }

    pub fn soft_reset() {
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

    fn parse_temp_calib_data(const *reg_data: u8) {
        T[0] = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
        T[1] = BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
        if(T[1] > 32767) { 
            T[1] -= 65536; 
        }
        T[2] = BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
        if(T[2] > 32767) { 
            T[2] -= 65536; 
        }
        
        P[0] = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
        for (int i = 0; i < 8; i++) {
            P[i+1] = reg_data[2*i+9]*256 + reg_data[2*i+8];
            if(P[i+1] > 32767) { 
                P[i+1] -= 65536; 
            }
        }
    }

    pub fn Process(&mut self) -> Result<(), LinuxI2CError> {   
        let mut comp_data: [u8; 6] = [0; 6];
        
        // read humidity
        self.Dev.smbus_write_byte(BMP280_DATA_ADDR);
        thread::sleep(Duration::from_millis(SLEEP_TIME));

        self.Dev.read(&mut comp_data);
        thread::sleep(Duration::from_millis(10));

        // Convert pressure and temperature data to 19-bits
        let adc_p = (comp_data[0] << 12) + (comp_data[1] << 4) + (comp_data[2] >> 4);
        let adc_t = (comp_data[3] << 12) + (comp_data[4] << 4) + (comp_data[5] >> 4);

        // temperature offset calculations
        let temp1 = adc_t / 16384.0 - (T[0]/1024.0)*T[1];
        let temp3 = adc_t / 131072.0 - (T[0]/8192.0);
        let temp2 = temp3 * temp3 * T[2];
        self.temperature = (temp1 + temp2) / 5120.0;

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
            Data->pressure = (press3 + (press1 + press2 + (P[6])) / 16.0) / 100;
        } else { 
            self.pressure = 0.0; 
        }
        self.pressure_nn = self.pressure / pow(1 - ALTITUDE/44330.0, 5.255);
    
        Ok(())
    }
}


