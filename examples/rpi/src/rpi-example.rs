use anyhow::Result;
use linux_embedded_hal::{Delay, I2cdev};


use bmp38x_ya::{BMP38x, constants::DeviceAddress, data::{Bmp3Configuration, PowerMode, FilterCoef, Odr, Over_Sampling}};

use std::thread;
use std::time::Duration;

use env_logger::Builder;
use log::{LevelFilter, error, info};
use std::io::Write;

fn main() -> Result<()> {

    let mut builder = Builder::from_default_env();

    builder
        .format(|buf, record| writeln!(buf, "{} - {}", record.level(), record.args()))
        .filter(None, LevelFilter::Info)
        .init();


    info!("Hello, linux Rust and bmp38x-ya world!");

    let dev_i2c = I2cdev::new("/dev/i2c-1").unwrap();
    let delayer = Delay {};

    let mut sensor = BMP38x::new(i2c_dev,DeviceAddress::Secondary as u8, &mut delay);
    log::info!("created BMP38x device, calling init_device next");
    sensor.init_device().unwrap();
    log::info!("BMP38x init_device done");

    let new_config = const {
        Bmp3Configuration::builder()
            .power_mode(PowerMode::Normal)  // very important!
            .temperature_enable(true)
            .pressure_enable(true)
            .over_sampling_temp(Over_Sampling::ULTRA_LOW_POWER) // ultra_low ok
            .over_sampling_press(Over_Sampling::HIGH_RESOLUTION)// standard ok
            .iir_filter_coef(FilterCoef::COEF_15) // CORF_15 ok
            .output_data_rate(Odr::ODR_12P5)  // ODR_50 ok,  ODR_12P5 for slower rate test
            .build()
    };

    sensor.set_bmp3_configuration(new_config).unwrap();
    let config = sensor.get_bmp3_configuration().unwrap();
    info!("new Bmp3Configuration = {:?}", config);
    hread::sleep(Duration::from_secs(1));



    loop {
        let status = sensor.get_status().unwrap();
        info!(" bmp38x status is  {:?}", status);
        if (status.get_temp_ready() && status.get_press_ready()) {
            let sensor_measurement = sensor.read_measurements_with_altitude(383.5).unwrap();  // current elevation is 383.5 m
            let intstatus = sensor.get_interrupt_status().unwrap();
            info!("  IntStatus is {:?}", intstatus);
            info!(" sensor_measurement = {:?}", sensor_measurement);
        } else {
            info!("  bmp38x data not ready");
        }
    }
    thread::sleep(Duration::from_secs(10));

    Ok(())


}
