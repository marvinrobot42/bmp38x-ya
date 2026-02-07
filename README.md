# BMP38x-ya &emsp; 
[![crates.io](https://img.shields.io/crates/v/bmp38x-ya)](https://crates.io/crates/bmp38x-ya)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](https://github.com/marvinrobot42/bmp38x-ya)
[![Documentation](https://docs.rs/si7021-t-rh/badge.svg)](https://docs.rs/si7021-t-rh)

## A Rust crate for Bosch Sensortec BMP38x and BMP390 air pressure sensor  (-ya = Yet Another)



fix <https://github.com/marvinrobot42/bmp38x-ya.git>

[BMP384]: https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp384/

The  Bosch Sensortec BMP 384 (388) and 390 air pressure and temperature sensor with I2C interface.

Most I2C API functions are implemented. 

### Features

- uses embedded-hal version 1.0.x
- async support included (see examples folder in repository for embassy-async ESP32-C6 example)
- designed for embedded use (ESP32-C3, -C6 and -S3 and Raspberry Pi)
- ESP32 RISC V and Raspberry Pi examples included
- most BMP384 IO functions implemented
- configurable interrupt pin function
- FIFO functions with configuration
- async functions involving I2C
- no_std embedded compatible

  

#### Notes

Developed using Sparkfun BMP384 (QWIIC) model: https://www.sparkfun.com/sparkfun-pressure-sensor-bmp384-qwiic.html


### Recent version history

  - 0.1.0  Initial release


## Usage
----

Add the dependency to `Cargo.toml`.

~~~~toml
[dependencies.bmp38x-ya]
version = "0.1"
~~~~

1. Create a hardward specific I²C driver interface and delay function  and a Delay object
2. Create an BMP38x struct with the I²C interface and a delay function as parameters.  
3. Initialize the BMP38x instance
4. Create a new Bmp3Configuration using either the handy const builder or initial the struct's properties as required
*    then call set_bmp3_configuration(new_bmp3Configuration)
1. Optional:  create a new InterruptPinControl instance and set its properties as required and call set_interrupt_pin_config fn
2. Optional:  create a new Fifo_Config instance and set its properties as required call set_fifo_config
3. call get_status() and if (status.get_temp_ready() && status.get_press_ready())
*    then call ither read_,easurements or read_measurements_with_altitude fn
 


### Simple Example

A more complete example is in the repository examples path
~~~~rust

use anyhow::Result;  // add dependency for this
use bmp38x_ya::{BMP38x, constants::DeviceAddress, data::{Bmp3Configuration, PowerMode, SensorFrameEnum}};
use log::{info, error};

...


fn main() -> Result<()> {

  ...as per your hardware hal, below is ESP32-C3 - C6 style

  let peripherals = Peripherals::take().unwrap();
  let pins = peripherals.pins;
  let sda = pins.gpio6; // esp32-c3  has pins.gpio0 , check your board schematic
  let scl = pins.gpio7; // esp32-c3  haspins.gpio1, check your board schematic
  let i2c = peripherals.i2c0;
  let config = I2cConfig::new().baudrate(400.kHz().into());
  let i2c_bus = I2cDriver::new(i2c, sda, scl, &config)?;

  let mut delay: Delay = Default::default();   // your hardware delay from use ...
  let mut sensor = BMP38x::new(i2c_dev,DeviceAddress::Secondary as u8, &mut delay);  // note the secondary I2C address
  sensor.init_device().unwrap();
  info!("BMP38x init_device done");
  let new_config = const {
        Bmp3Configuration::builder()
            .power_mode(PowerMode::Normal)  //  *** very important to get measurements
            .temperature_enable(true)
            .pressure_enable(true)
            .over_sampling_temp(Over_Sampling::ULTRA_LOW_POWER) // ultra_low ok
            .over_sampling_press(Over_Sampling::HIGH_RESOLUTION)// standard ok
            .iir_filter_coef(FilterCoef::COEF_15) // CORF_15 ok
            .output_data_rate(Odr::ODR_12P5)  // ODR_50 ok,  ODR_12P5 for slower rate testing fifo operation
            .build()
    };
    sensor.set_bmp3_configuration(new_config).unwrap();
    let config = sensor.get_bmp3_configuration().unwrap();  // optional just to see that set config worked
    info!("new Bmp3Configuration = {:?}", config);

  FreeRtos::delay_ms(250);  // or your sleep function here

  loop {
    let status = sensor.get_status().unwrap();
    info!(" bmp38x status is  {:?}", status);
    if (status.get_temp_ready() && status.get_press_ready()) {
      let sensor_measurements = sensor.read_measurements_with_altitude(383.5).unwrap();
      info!(" sensor measurements = {:?}", sensor_measurements);
    } else {
            log::info!("  bmp38x sensor data not ready!");
    }
    FreeRtos::delay_ms(10000);
  }

}
    
~~~~

### For async set bmp38x-ya dependency features = ["async"] and Bmp38x::new method requires async I2C and delay 
###    parameters.  Default features is sync (blocking)


### License
----

You are free to copy, modify, and distribute this application with attribution under the terms of either

 * Apache License, Version 2.0
   ([LICENSE-Apache-2.0](./LICENSE-Apache-2.0) or <https://opensource.org/licenses/Apache-2.0>)
 * MIT license
   ([LICENSE-MIT](./LICENSE-MIT) or <https://opensource.org/licenses/MIT>)

at your option.

This project is not affiliated with nor endorsed in any way by Silicon Labs or Adafruit.
