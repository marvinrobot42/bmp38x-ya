use anyhow::Result;

// ESP32-C6 std example

use esp_idf_hal::{
    delay::{Ets, FreeRtos}, 
    gpio::*,
    i2c::{I2cConfig, I2cDriver}, io::Error, 
    prelude::*
};
// use esp_idf_hal::{gpio::PinDriver, prelude::Peripherals};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::peripheral;

use esp_idf_hal::delay::Delay;

use embedded_hal_bus::i2c;


use bmp38x_ya::{BMP38x, constants::DeviceAddress, data::{Bmp3Configuration, Fifo_Config, FifoConfig1, FifoConfig2, FifoDataSelect, FifoSubSampling, FilterCoef, IntActiveState, IntOutputMode, InterruptPinControl, Odr, Over_Sampling, PowerMode}};

use esp_idf_sys::{self as _, esp_image_flash_size_t_ESP_IMAGE_FLASH_SIZE_128MB};
use log::{info, error, debug};


use libm::*;

use esp_idf_svc::sys::EspError;
use esp_idf_svc::wifi::*;



fn main() -> Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Hello, BMP38x-YA Test, version 0.1.0!");


    let peripherals = Peripherals::take().unwrap();

    let pins = peripherals.pins;
    let sda = pins.gpio6; // esp32-c3  has pins.gpio0;
    let scl = pins.gpio7; // esp32-c3  haspins.gpio1;
    let i2c = peripherals.i2c0;
    let config = I2cConfig::new().baudrate(400.kHz().into());
    let i2c_dev = I2cDriver::new(i2c, sda, scl, &config)?;

    let mut led_pin23 = PinDriver::output(pins.gpio23).unwrap();
    led_pin23.set_high().unwrap();



     //let sysloop = EspSystemEventLoop::take()?;
     let sys_loop = EspSystemEventLoop::take().unwrap();
     info!("got sys_loop");


    // BMP38x-ya stuff ************************************************************************
    
    let mut delay: Delay = Default::default();
    log::info!("creating BMP38x (384?) device");
    let mut sensor = BMP38x::new(i2c_dev,DeviceAddress::Secondary as u8, &mut delay);
    log::info!("created BMP38x device, calling init_device next");
    sensor.init_device().unwrap();
    log::info!("BMP38x init_device done");
    let pwr_mode = sensor.get_power_mode().unwrap();
    info!("  power control mode = {:?}", pwr_mode);

    let config = sensor.get_bmp3_configuration().unwrap();
    info!("initial Bmp3Configuration = {:?}", config);

    // create a new config using basic struct initialization
    // let new_config= Bmp3Configuration {
    //     power_mode:  PowerMode::Normal,
    //     temperature_enable: true,
    //     pressure_enable: true,
    //     over_sampling_temp: Over_Sampling::HIGH_RESOLUTION,
    //     over_sampling_press: Over_Sampling::STANDARD_RESOLUTION,
    //     output_data_rate: Odr::ODR_0P02,
    //     iir_filter_coef: FilterCoef::COEF_15,
    // };


    // note:  the Bmp3Configuration struct uses a const_builder so the interrupt pin config cannot be included
    //        because it is a bitfield which cannot be a const struct due to its setter methods
    //        the below configuration results a slow update rate for testing fifo operation in loop
    let new_config = const {
        Bmp3Configuration::builder()
            .power_mode(PowerMode::Normal)
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
    
    
    // configurae the BMP38x interrupt pin, but not used in this example
    let mut intpin_config: InterruptPinControl = InterruptPinControl(0x00);
    intpin_config.set_active_state(true);  // high == true
    intpin_config.set_latching(true);     // latching (true) works 
    intpin_config.set_output_mode(false); // push_pull (false)  works with ESP32 pin as floating input
    intpin_config.set_temp_press_rd(true);  // was true, data ready sets pin as active
    intpin_config.set_fifo_full(false);  // false == overwrite old data
    intpin_config.set_fifo_wm(false);
    sensor.set_interrupt_pin_config(intpin_config).unwrap();
    let new_intpin_config = sensor.get_interrupt_pin_configuration().unwrap();
    info!(" new interrupt pin config is {:?}", new_intpin_config);
    // let my_result = sensor.fifo_flush();
    // match my_result {
    //     Ok(()) => info!(" ok"),
    //     Err(_error) => info!(" error"),
    // }

    // create a FIFO configuration with slow update rate for testing
    let fifo_config = sensor.get_fifo_config().unwrap();
    info!(" initial fifo config is {:?}", fifo_config);
    let mut new_fif_config_1 : FifoConfig1 = FifoConfig1(0x00);
    new_fif_config_1.set_fifo_enable(true);
    new_fif_config_1.set_fifo_press_enable(true);
    new_fif_config_1.set_fifo_temp_enable(true);
    new_fif_config_1.set_fifo_time_enable(true);  // probably not needed
    let mut new_fifo_config_2: FifoConfig2 = FifoConfig2(0x00);
    new_fifo_config_2.set_fifo_data_select(FifoDataSelect::UNFILTERED as u8);
    new_fifo_config_2.set_fifo_subsampling(FifoSubSampling::SIXTYFOUR as u8);  // slower update rate with SIXTYFOUR
    let mut new_fifo_config: Fifo_Config = Fifo_Config { fifo_config_1: new_fif_config_1, fifo_config_2: new_fifo_config_2 };
    sensor.set_fifo_config(new_fifo_config).unwrap();
    let fifo_config = sensor.get_fifo_config().unwrap();  // read it back to see that change worked
    info!(" new fifo config is {:?}", fifo_config);

    sensor.fifo_set_watermark_frames(10).unwrap();  // used in WM interrupt when configurated
    let wm_frames = sensor.get_fifo_watermark_in_frames().unwrap();
    info!(" watermark in frames is {}", wm_frames);

    info!("done bmp38x setting");

    FreeRtos::delay_ms(1000);
    led_pin23.set_low().unwrap();


    loop {

        // read bmp38x data directly ***************************************
        let status = sensor.get_status().unwrap();
        info!(" bmp38x status is  {:?}", status);
        if (status.get_temp_ready() && status.get_press_ready()) {
            led_pin23.set_high().unwrap();
            let sensor_measurement = sensor.read_measurements_with_altitude(383.5).unwrap();
            let intstatus = sensor.get_interrupt_status().unwrap();
            info!("  IntStatus is {:?}", intstatus);
            info!(" sensor_measurement = {:?}", sensor_measurement);
        } else {
            info!(" bmp38x data not ready");
        }
        // end of read bmp38x data directly *********************************

        // read FIFO data.  if not using fifo comment out below **********************
        let mut num_frames = sensor.get_fifo_number_frames().unwrap();
        info!(" number of frames = {}", num_frames);

        if (num_frames > 0)  {
            for _number in 1..=num_frames {
                let frame = sensor.get_fifo_frame().unwrap();
                info!(" fifo_frame = {:?}", frame);
            }
        } else {
            info!(" fifo is empty");
        }
        // end of fifo read **********************************************************

        FreeRtos::delay_ms(10000); // every 10 seconds, results is about 3 fifo measurements each loop
        led_pin23.toggle().unwrap();
    }

    //Ok(())

}
