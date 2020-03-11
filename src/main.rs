extern crate i2cdev;
use i2cdev::linux::LinuxI2CDevice;

use std::thread;
use std::time::Duration;
//use std::io::prelude::*;

use gpio::GpioOut;
use rumqtt::{MqttClient, MqttOptions, QoS};

// ------------------------------------------------------------------------------
// HTU21D stuff
// ------------------------------------------------------------------------------
mod libs;
use crate::libs::veml6070::VEML6070;

const OFF: u8 = 0;
const ON: u8 = 1;

// ------------------------------------------------------------------------------
// GPIO stuff
// ------------------------------------------------------------------------------
fn toggle_leds(val: f32) {
    // Let's open GPIO23 and -24, e.g. on a Raspberry Pi 2.
    let mut green = gpio::sysfs::SysFsGpioOutput::open(20).unwrap();
    let mut red = gpio::sysfs::SysFsGpioOutput::open(26).unwrap();

    /* GPIO24 will be toggled every second in the background by a different thread
    let mut value = false;
    thread::spawn(move || loop {
        green.set_value(value).expect("could not set gpio20");
        thread::sleep(Duration::from_millis(1000));
        value = !value;
    });

    // The main thread will simply display the current value of GPIO23 every 100ms.
    loop {
        red.set_value(!value).expect("could not set gpio26");
        thread::sleep(Duration::from_millis(100));
        value = !value;
    }*/
    match val {
        19.5 ... 25.5 => {
            green.set_value(ON).expect("could not set gpio20");
            red.set_value(OFF).expect("could not set gpio26");
        },
        0.0 ...19.4 => {
            green.set_value(OFF).expect("could not set gpio20");
            red.set_value(ON).expect("could not set gpio26");  
        },
        25.6 ... 33.0 => {
            green.set_value(OFF).expect("could not set gpio20");
            red.set_value(ON).expect("could not set gpio26");
        },
        _ => println!("{:?}", val),
    }
}

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
    let mut bmp: BMPSensor = libs::bmp280::BMPSensor::new("/dev/i2c-1");
    
    thread::spawn(move || loop {
        htu21_sen.Process();
        bmp.Process();
        
        let uv = uvSensor.ReadUV();
        println!("BMP Temp: {:?}", bmp.temperature);
        println!("BMP Pres: {:?}", bmp.pressure);

        let payload = format!("{{\"Oben\": {{\"Temperature\":{}, \"Humidity\":{}, \"UV\":{}}}}}", htu21_sen.Temperatur, htu21_sen.Humidity, uv);
        mqtt_client.publish("sensor/oben", QoS::AtLeastOnce, false, payload).unwrap();
        
        toggle_leds(htu21_sen.Temperatur);
        // at least sleep for 5 seconds
        thread::sleep(Duration::from_millis(60000));
    });

    for notification in notifications {
        println!("{:?}", notification)
    }
}
