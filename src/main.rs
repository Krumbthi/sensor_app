use linux_embedded_hal::{Delay, I2cdev};

use std::{thread, process};
use std::time::Duration;

use rumqttc::{MqttOptions, Client, QoS, Event, Packet};
use log::{info, debug, error};
use env_logger::Env;


// ------------------------------------------------------------------------------
// HTU21D stuff
// ------------------------------------------------------------------------------
//mod libs;
use create::bme280::BME280;

// ------------------------------------------------------------------------------
// Main loop
// ------------------------------------------------------------------------------
fn main() {
    let env = Env::filter_or(Env::default(), "APP_LOG_LEVEL", "debug")
        .write_style_or("APP_LOG_STYLE", "always");    
    env_logger::init_from_env(env);

    info!("-------------------------------");
    info!("| Interface test with Rust :) |");
    info!("-------------------------------");

    let mut mqtt_options = MqttOptions::new("pi-oben", "192.168.2.2", 1883);
    mqtt_options.set_keep_alive(Duration::from_secs(5));

    let (mut mqtt_client, mut connection) = Client::new(mqtt_options, 10);
    mqtt_client.subscribe("sensor/rpi0", QoS::AtLeastOnce).unwrap();

    
    let i2c_bus = I2cdev::new("/dev/i2c-1").unwrap();
    let mut bme280 = BME280::new_primary(i2c_bus, Delay);
    bme280.init().unwrap();

    thread::spawn(move || loop {
        let meas = bme280.measure().unwrap();
        let pres = meas.pressure / 100.0;

        debug!("Bmp280 pressure: {:?} kPa", pres);
        debug!("Bmp280 humidity: {:?} %", meas.humidity);
        debug!("Bmp280 temperature: {:?} c", meas.temperature);

        let payload = format!("{{\"Rpi\": {{\"Temperature\":{}, \"Humidity\":{}, \"Pressure\":{} }}}}", meas.temperature, meas.humidity, pres);
        mqtt_client.publish("sensor/rpi0", QoS::AtLeastOnce, false, payload).unwrap();

        // at least sleep for 5 seconds
        thread::sleep(Duration::from_millis(360000));
    });

    for (i, notification) in connection.iter().enumerate() {
        match notification {
            Ok(Event::Incoming(i)) => {
                //debug!("Incomming: {:?}", i);
                match i {
                    Packet::Publish(p) => {
                        info!("Payload: {:?}", p.payload);
                        //if p.payload.to_vec() == "exit" {
                        if p.payload.slice(0..4) == "exit" {
                            process::exit(0);
                        }
                    },
                    _ => (),
                }
            }
            //Ok(Event::Outgoing(o)) => debug!("Outgoing: {:?}", o),
            Err(e) => error!("Error = {:?}", e),
            _ => (),
        }
    }
}
