use std::thread;
use std::time::Duration;

use i2cdev::core::*;
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};

// ------------------------------------------------------------------------------
// Defines
// ------------------------------------------------------------------------------
pub const SLAVE_ADDR_PRIMARY: u16 = 0x40;

/* SMBus version */
const READ_TEMP_HOLD_MASTER_CMD: u8 = 0xE3;
const READ_HUM_HOLD_MASTER_CMD: u8 = 0xE5;
const READ_TEMP_NO_HOLD_MASTER_CMD: u8 = 0xF3;
const READ_HUM_NO_HOLD_MASTER_CMD: u8 = 0xF5;
const WRITE_USER_CMD: u8 = 0xE6;
const READ_USER_CMD: u8 = 0xE7;
const SOFT_RESET_CMD: u8 = 0xFE;
const SLEEP_TIME: u64 = 55;

// ------------------------------------------------------------------------------
// Globals
// ------------------------------------------------------------------------------

/*pub trait Sensor {
    fn calc_humidity(&self, value: u16) -> f32;
    fn calc_temperatur(&self, value: u16) -> f32;
    fn Setup(&mut self);
    fn Process(&mut self) -> Result<(), LinuxI2CError>;
}
*/

pub struct HTU21Sensor {
    pub Temperatur: f32,
    pub Humidity: f32,
    pub Dev: LinuxI2CDevice,
}

//impl Sensor for HTU21Sensor {
impl HTU21Sensor {
    fn calc_humidity(&self, value: u16) -> f32 {
        //-6.0 + 125.0*((value & !0x3) as f32)/((1 << 16) as f32)
        let tSensorHumid = (value as f32) / 65536.5;
        -6.0 + (125.0 * tSensorHumid)
    } 

    fn calc_temperatur(&self, value: u16) -> f32 {
        //-46.85 + 175.72 * ((value & !0x3) as f32) / ((1 << 16) as f32)
        let tSensorTemp = (value as f32) / 65536.0;
        -46.85 + (175.72 * tSensorTemp)
    }

    pub fn Setup(&mut self) {
        // soft reset
        self.Dev.smbus_write_byte(SOFT_RESET_CMD);
        thread::sleep(Duration::from_millis(100));
        println!("Soft Reset done!");
    }

    pub fn Process(&mut self) -> Result<(), LinuxI2CError> {   
        let mut buf: [u8; 3] = [0; 3];
        
        // read humidity
        self.Dev.smbus_write_byte(READ_HUM_HOLD_MASTER_CMD);
        thread::sleep(Duration::from_millis(SLEEP_TIME));

        self.Dev.read(&mut buf);
        thread::sleep(Duration::from_millis(10));

        let hum = ((buf[0] as u16) << 8) | (buf[1] as u16);
        self.Humidity = self.calc_humidity(hum);
        println!("Humidity: {:?} %", self.Humidity); 

        // read Temperatur
        self.Dev.smbus_write_byte(READ_TEMP_NO_HOLD_MASTER_CMD);
        thread::sleep(Duration::from_millis(SLEEP_TIME));

        self.Dev.read(&mut buf);
        thread::sleep(Duration::from_millis(10));	
        let temp = ((buf[0] as u16) << 8) | (buf[1] as u16);
        self.Temperatur = self.calc_temperatur(temp);

        println!("Temperatur: {:?} C", self.Temperatur);
    
        Ok(())
    }
}
