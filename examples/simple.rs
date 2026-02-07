// ESP32-C6 based example

/**** Cargo.toml includes:

[dependencies]
log = { version = "0.4", default-features = false }
embedded-svc = "0.25.0"
esp-idf-svc = { version = "0.48", default-features = false }
esp-idf-sys = { version = "0.34.0", features = ["binstart"] }
esp-idf-hal = "0.43.0"

anyhow = "1.0.75"
bmp38x-ya = "0.1"

[build-dependencies]
embuild = "0.31.3"

****************/

use anyhow::Result;

use esp_idf_hal::{
    delay::{Ets, FreeRtos}, i2c::{I2cConfig, I2cDriver}, io::Error, prelude::*
};
use esp_idf_hal::{gpio::PinDriver, prelude::Peripherals};
use esp_idf_hal::peripheral;

use esp_idf_hal::delay::Delay;


use esp_idf_sys::{self as _};
use log::{info, error};

use bmp38x_ya::{BMP38x, constants::DeviceAddress, data::{Bmp3Configuration, PowerMode, FilterCoef, Odr, Over_Sampling}};


fn main() -> Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Hello, bmp38x-ya world!");

    let peripherals = Peripherals::take().unwrap();
    let pins = peripherals.pins;
    let sda = pins.gpio6; // esp32-c3  has pins.gpio0 , check your board schematic
    let scl = pins.gpio7; // esp32-c3  haspins.gpio1, check your board schematic
    let i2c = peripherals.i2c0;
    let config = I2cConfig::new().baudrate(400.kHz().into());
    let i2c_bus = I2cDriver::new(i2c, sda, scl, &config)?;

    
    let mut delay: Delay = Default::default();
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

    // if you only need ULTRA_LOW_POWER or less resolution you can just set the power mode to normal with
    //  sensor.set_power_control(PowerControl::normal()).unwrap() and skip the set_bmp3_configuration call

    sensor.set_bmp3_configuration(new_config).unwrap();
    let config = sensor.get_bmp3_configuration().unwrap();  // read it back to see that it worked
    info!("new Bmp3Configuration = {:?}", config);
    FreeRtos::delay_ms(1000);



    loop {
        let status = sensor.get_status().unwrap();
        info!(" bmp38x status is  {:?}", status);
        if (status.get_temp_ready() && status.get_press_ready()) {  // without checking this you could get old data
            let sensor_measurement = sensor.read_measurements_with_altitude(383.5).unwrap();  // current elevation is 383.5 m
            // or let sensor_measurement = sensor.read_measurements().unwrap();  if sealevel pressure not used
            let intstatus = sensor.get_interrupt_status().unwrap();
            info!("  IntStatus is {:?}", intstatus);
            info!(" sensor_measurement = {:?}", sensor_measurement);
        } else {
            info!("  bmp38x data not ready");
        }
    }

    Ok(())

}
