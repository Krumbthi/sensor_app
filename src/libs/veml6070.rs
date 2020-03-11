use std::collections::HashMap;

use i2cdev::core::*;
use i2cdev::linux::LinuxI2CDevice;

/* --------------------------------------------------
    Constants
   --------------------------------------------------*/
const ADDR_L: u8 = 0x38; // 7bit address of the VEML6070 (write, read)
const ADDR_H: u8 = 0x39; // 7bit address of the VEML6070 (read)

const RSET_240K: i32 = 240000;
const RSET_270K: i32 = 270000;
const RSET_300K: i32 = 300000;
const RSET_600K: i32 = 600000;

const SHUTDOWN_DISABLE: u8 = 0x00;
const SHUTDOWN_ENABLE: u8 = 0x01;

const INTEGRATIONTIME_1_2T: u8 = 0x00;
const INTEGRATIONTIME_1T: u8 = 0x01;
const INTEGRATIONTIME_2T: u8 = 0x02;
const INTEGRATIONTIME_4T: u8 = 0x03;

/* --------------------------------------------------
    Struct 
   --------------------------------------------------*/
pub struct VEML6070 {
	LightIntensity: f32,
    IntegrationTime: u8,
    Rset: i32,
    Shutdown: u8,
    CaseSensIt: HashMap<u8, f32>,
    Dev: LinuxI2CDevice,
}

pub trait UVSensor {
    fn new(dev_name: &'static str) -> Self;
    fn getUV(&mut self) -> u16;
}

impl VEML6070 {
    /*
    fn Setup(&mut self) {
        //self.CaseSensIt = HashMap::new();
        self.CaseSensIt.insert(INTEGRATIONTIME_1_2T, 0.5);
        self.CaseSensIt.insert(INTEGRATIONTIME_1T, 1.0);
        self.CaseSensIt.insert(INTEGRATIONTIME_2T, 2.0);
        self.CaseSensIt.insert(INTEGRATIONTIME_4T, 4.0);
        
        self.Shutdown = SHUTDOWN_DISABLE; // before set_integration_time()
        self.SetIntegrationTime(INTEGRATIONTIME_1T);
        self.Disable();
    }

	fn Disable(&mut self) {
        self.Shutdown = SHUTDOWN_ENABLE;
        self.Dev.smbus_write_byte(self.GetCmdByte());
	}

	fn Enable(&mut self) {
		self.Shutdown = SHUTDOWN_DISABLE;
        self.Dev.smbus_write_byte(self.GetCmdByte());
	}
    
    fn SetIntegrationTime(&mut self, intTime: u8) {
        self.IntegrationTime = intTime;
        self.Dev.smbus_write_byte(self.GetCmdByte());
        // constant offset determined experimentally to allow sensor to readjust
        thread::sleep(Duration::from_millis(200));
    }

	fn GetUvaLightIntensityRaw(&mut self) -> u16 {
        self.Enable();
        // wait two times the refresh time to allow completion of a previous cycle with old settings (worst case)
        thread::sleep(Duration::from_millis((self.GetRefreshTime() * 2.0) as u64));
        
        let mut buf: [u8; 2] = [0; 2];
        self.Dev.read(&mut buf);
        let data: u16 = ((buf[0] as u16) << 8) | buf[1] as u16;
        self.Disable();
        
        data
	}

    fn GetUvaLightSensitivity(&mut self) -> f32 {
        // returns UVA light sensitivity in W/(m*m)/step
        let mut caseSensRset = HashMap::new();
        caseSensRset.insert(RSET_240K, 0.05);
        caseSensRset.insert(RSET_270K, 0.05625);
        caseSensRset.insert(RSET_300K, 0.0625);
        caseSensRset.insert(RSET_600K, 0.125);

        caseSensRset[&self.Rset] / self.CaseSensIt[&self.IntegrationTime]
    }

	pub fn GetUvaLightIntensity(&mut self) -> u16 {
        let uv = self.GetUvaLightIntensityRaw();
        uv * self.GetUvaLightSensitivity()
	}

	fn GetCmdByte(&mut self) -> u8 {
        // assembles the command byte for the current state
        let mut cmd = (self.Shutdown & 0x01) << 0;  // SD
        cmd = (self.IntegrationTime & 0x03) << 2;   // IT
        ((cmd | 0x02) & 0x3F)                       // reserved bits
	}

	fn GetRefreshTime(&mut self) -> f32 {
        let mut caseSensRset = HashMap::new();
        caseSensRset.insert(RSET_240K, 0.1);
        caseSensRset.insert(RSET_270K, 0.1125);
        caseSensRset.insert(RSET_300K, 0.125);
        caseSensRset.insert(RSET_600K, 0.25);

        caseSensRset[&self.Rset] / self.CaseSensIt[&self.IntegrationTime]
    }
    */
    pub fn ReadUV(&mut self) -> u16 {
        let mut buf: [u8; 1] = [0; 1];
        
        self.Dev.smbus_write_byte(ADDR_H);
        self.Dev.read(&mut buf);

        let data: u16 = (buf[0] as u16) << 8;
        
        self.Dev.smbus_write_byte(ADDR_L);
        self.Dev.read(&mut buf);

        let uvi = data | (buf[0] as u16);
        println!("UV: {:?}", uvi);
        uvi
    }
}

impl UVSensor for VEML6070 {
    fn new(dev_name: &'static str) -> VEML6070 {
        VEML6070 { 
            LightIntensity: 0.0,
            IntegrationTime: INTEGRATIONTIME_1T,
            Rset: RSET_270K,
            Shutdown: SHUTDOWN_DISABLE,
            CaseSensIt: HashMap::new(),
            Dev: LinuxI2CDevice::new(dev_name, ADDR_L.into()).unwrap()
        }
    }

    fn getUV(&mut self) -> u16 {
        self.ReadUV()
    }
}