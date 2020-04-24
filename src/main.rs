extern crate i2cdev;

use i2cdev::linux::LinuxI2CDevice;

use std::thread;
use std::time::Duration;
//use std::io::prelude::*;

//use gpio::GpioOut;
use rumqtt::{MqttClient, MqttOptions, QoS};

// ------------------------------------------------------------------------------
// HTU21D stuff
// ------------------------------------------------------------------------------
mod libs;
use crate::libs::bmp280::Bmp280Builder;
use crate::libs::veml6070::VEML6070;

// ------------------------------------------------------------------------------
// Main loop
// ------------------------------------------------------------------------------
fn main() {
    println!("-------------------------------");
    println!("| Interface test with Rust :) |");
    println!("-------------------------------");

    let mqtt_options = MqttOptions::new("test-pubsub1", "192.168.2.2", 1883);
    let (mut mqtt_client, notifications) = MqttClient::start(mqtt_options).unwrap();

    mqtt_client.subscribe("sensor/oben", QoS::AtLeastOnce).unwrap();
        
    //    read_mcp();
    let mut htu21_sen = libs::htu21::HTU21Sensor{ Temperatur: 0.0, Humidity: 0.0, Dev: LinuxI2CDevice::new("/dev/i2c-1", libs::htu21::SLAVE_ADDR_PRIMARY).unwrap() };
    let mut uvSensor: VEML6070 = libs::veml6070::UVSensor::new("/dev/i2c-1");
    
    let mut bmp280 = Bmp280Builder::new()
        .path("/dev/i2c-1".to_string())
        .address(0x76)
        .build()
        .expect("Failed to build device");

    bmp280.zero().expect("failed to zero");
    
    thread::spawn(move || loop {
        htu21_sen.Process();
        let uv = uvSensor.ReadUV();
        let bmp_temp = bmp280.read_temperature().unwrap();
        let bmp_pres = bmp280.read_pressure().unwrap()/100.0;
        let bmp_alt = bmp280.read_altitude().unwrap();
        
        let temp = (htu21_sen.Temperatur + bmp_temp) / 2.0;
        
        println!("{:?} hPa", bmp280.read_pressure().unwrap());
        println!("{:?} m", bmp280.read_altitude().unwrap());
        println!("{:?} c", temp);

        let payload = format!("{{\"Oben\": {{\"Temperature\":{}, \"Humidity\":{}, \"Pressure\":{}, \"Altitude\":{}, \"UV\":{}}}}}", temp, htu21_sen.Humidity, bmp_pres, bmp_alt, uv);
        mqtt_client.publish("sensor/oben", QoS::AtLeastOnce, false, payload).unwrap();
        
        // at least sleep for 5 seconds
        thread::sleep(Duration::from_millis(60000));
    });

    for notification in notifications {
        println!("{:?}", notification)
    }
}
