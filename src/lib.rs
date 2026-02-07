#![no_std]
#![allow(dead_code)]
#![allow(unused_variables)]

//#![feature(inherent_associated_types)]
pub mod error;


use crate::data::{Bmp3Configuration, ControlFrameEnum, FifoConfig1, FifoConfig2, SensorFrameEnum, TandPtype};
use crate::error::Error;

pub mod data;
// ojs  use crate::data::{DeviceInfo, DeviceModel, CalibrationPars};

pub mod constants;

use crate::constants::DeviceAddress::{Primary, Secondary};



use constants::{BMP3_REG_CHIP_ID, BMP3_REG_ERR, BMP3_REG_SENS_STATUS, BMP3_REG_PRESS_DATA, BMP3_REG_TEMP_DATA,
                BMP3_REG_SENSOR_TIME, BMP3_REG_EVENT, BMP3_REG_INT_STATUS, BMP3_REG_FIFO_LENGTH, BMP3_REG_FIFO_DATA,
                BMP3_REG_FIFO_WM, BMP3_REG_FIFO_CONFIG_1, BMP3_REG_FIFO_CONFIG_2, BMP3_REG_INT_CTRL, 
                BMP3_REG_IF_CONF, BMP3_REG_PWR_CTRL, BMP3_REG_OSR, BMP3_REG_ODR, BMP3_REG_CONFIG, BMP3_REG_CALIB_DATA,
                BMP3_REG_CMD, BMP3_SOFT_RESET, BMP3_FIFO_FLUSH};
                
use data::{DeviceInfo, DeviceModel, Measurements, PWR_CTRL, PowerMode, Status, ErrorReg, IntStatus, Odr, Over_Sampling,
           OSR, FilterCoef, CalibrationPars, InterruptPinControl, Fifo_Config, FifoFrame};

#[cfg(not(feature = "async"))]
use embedded_hal::{i2c::I2c, delay::DelayNs};
#[cfg(feature = "async")]
use embedded_hal_async::{i2c::I2c as AsyncI2c, delay::DelayNs as AsyncDelayNs};

use log::{debug, info};
use libm::{pow, truncf};

const STANDARD_SEA_LEVEL_AIR_PRESSURE: f64 = 101325.0;  // Pa


/// the BMP38x device
pub struct BMP38x<I2C, D> {
    /// I²C interface
    i2c: I2C,
    /// I²C device address
    address: u8,
    delayer: D,
    pub device_info: DeviceInfo,
    cal_pars: CalibrationPars,
    fifo_config:  Fifo_Config,
    /// altitude in metres above sealevel, used for sealevel equivalent air pressure calculation
    pub altitude_m: f64,
}

#[cfg(not(feature = "async"))]
impl<I2C, D, E> BMP38x<I2C, D>
where  
    I2C: I2c<Error = E>,
    D: DelayNs,
{
    
    //type Error = Error;
    /// create new BMP38x driver with default I2C address: ADDR pin low
    pub fn new(i2c: I2C, address: u8, delayer: D) -> Self {
        log::debug!("new called");
        Self {
            i2c,
            address: address,
            delayer,
            device_info: DeviceInfo { ..Default::default()},
            cal_pars: CalibrationPars { .. Default::default()},
            fifo_config:  Fifo_Config { .. Default::default()},
            altitude_m : 0.0,
        }
    }

    pub fn new_with_altitude(i2c: I2C, address: u8, delayer: D, altitude: f64) -> Self {
        log::debug!("new_with_altitude called");
        Self {
            i2c,
            address: address,
            delayer,
            device_info: DeviceInfo { ..Default::default()},
            cal_pars: CalibrationPars { .. Default::default()},
            fifo_config:  Fifo_Config { ..Default::default()},
            altitude_m : altitude,
        }
    }

    /// give back the I2C interface
    pub fn release(self) -> I2C {
        self.i2c
    }

}

#[cfg(feature = "async")]
impl<I2C, D, E> BMP38x<I2C, D>
where  
    I2C: AsyncI2c<Error = E>,
    D: AsyncDelayNs,
{
    //type Error = Error;
    /// create new BMP38x driver with default I2C address: ADDR pin low
    pub fn new(i2c: I2C, address: u8, delayer: D) -> Self {
        debug!("new called");
        Self {
            i2c,
            address: address,
            delayer,
            device_info: DeviceInfo { ..Default::default()},
            cal_pars: CalibrationPars::default(),
        }
    }

    /// give back the I2C interface
    pub fn release(self) -> I2C {
        self.i2c
    }

}

#[maybe_async_cfg::maybe(
    sync(
        cfg(not(feature = "async")),
        self = "BMP38x",
        idents(AsyncI2c(sync = "I2c"), AsyncDelayNs(sync = "DelayNs"))
    ),
    async(feature = "async", keep_self)
)]

impl<I2C, D, E> BMP38x<I2C, D>
where  
    I2C: AsyncI2c<Error = E>,
    D: AsyncDelayNs,
{

    // command_buf is an u8 array that starts with command byte followed by command data byte(s)
    async fn write_command<const N: usize>(&mut self, command_buf: [u8; N] ) -> Result<(), Error<E>> {
        // debug!("write_command : {:#?}", command_buf);
        self.i2c
            .write(self.address, &command_buf).await
            .map_err(Error::I2c)?;
        Ok(())
    }

    async fn read_register( &mut self, register_address: u8, buffer: &mut [u8] ) -> Result<(), Error<E>> {
        let mut command_buffer = [0u8; 1];
        command_buffer[0] = register_address;
        // let mut result_buffer = [0u8; N];
        self.i2c
            .write_read(self.address, &command_buffer, buffer).await
            .map_err(Error::I2c)?;
        Ok(())
    }




    /// reset BMP38x soft reset 
    pub async fn reset_device(&mut self) -> Result<(), Error<E>> {
        debug!("in reset_device()");
        // check for (new) command ready
        let mut result_buf: [u8; 1] = [0; 1];
        self.read_register(BMP3_REG_SENS_STATUS, &mut result_buf).await?;
        let status = Status(result_buf[0]);
        if (status.get_cmd_ready()) {
            //let write_buf: [u8; 2] = [BMP3_REG_CMD, BMP3_SOFT_RESET];
            self.write_command([BMP3_REG_CMD, BMP3_SOFT_RESET]).await?;

            // self.i2c.write(self.address, &write_buf).await
            //         .map_err(Error::I2c);

            //self.write_command([BMP3_REG_CMD, write_buf]).await? ; 
            self.delayer.delay_ms(5).await;
            self.read_register(BMP3_REG_EVENT, &mut result_buf).await?;  // clear the stuff
            self.read_register(BMP3_REG_ERR, &mut result_buf).await?;
            let error_value = ErrorReg(result_buf[0]);
            if ( error_value.get_cfg_error() || error_value.get_cmd_error() || error_value.get_fatal_error() ) {
                return Err(Error::CommandError);
            } 
            return Ok(());
        }
        Err(Error::NotReadyForCommand)   
    }


    /// initial BMP38x device, setting Normal PWR_CTRL mode, and enabling pressure and temperature sensing
    ///                        also, using default output data rates
    pub async fn init_device(&mut self) -> Result<bool, Error<E>> {
        // set default calibration values
        // check chip ID
        debug!("in init_device(), doing reset_device");
        self.reset_device().await?;
        //#[allow(unused_parens)]

        if (self.get_chip_id().await? == DeviceModel::NotRead) {
            debug!("device model is not read");
            return Ok(false);
        }
        debug!("reading calibration pars");
        self.read_calibration_pars()?;
        self.set_power_control_mode(PowerMode::Normal)?;
        let pwr_ctrl_now = self.get_power_mode().await?;
        debug!("  power_control_mode is now {:?}", pwr_ctrl_now);
        self.set_pressure_enable(true)?;
        self.set_temperature_enable(true)?;
        let pwr_ctrl_now = self.get_power_mode().await?;
        debug!("  power_control_mode is after press and temp enabled {:?}", pwr_ctrl_now);
        Ok(true)
    }


    /// read device model 
    pub async fn get_chip_id(&mut self) -> Result<DeviceModel, Error<E>> {
        let mut result_buf: [u8; 1] = [0; 1];
        let mut command_buffer: [u8; 1] = [BMP3_REG_CHIP_ID];
        self.i2c
            .write_read(self.address, &command_buffer, &mut result_buf).await
            .map_err(Error::I2c)?;
        self.device_info.id = DeviceModel::from(result_buf[0]);
          
        Ok(DeviceModel::from(result_buf[0]))
    }

    /// get status
    pub async fn get_status(&mut self) -> Result<Status, Error<E>> {
        debug!("in get_status()");
        // check for (new) command ready
        let mut result_buf: [u8; 1] = [0; 1];
        self.read_register(BMP3_REG_SENS_STATUS, &mut result_buf).await?;
        //info!("  get_status returned {:#b}", result_buf[0]);
        let status = Status(result_buf[0]);
        Ok(status)
    }

    /// set Power Mode
    pub async fn set_power_mode(&mut self, power_mode: PowerMode) -> Result<(), Error<E>> {
        debug!("in set_power_mode( {:?} )", power_mode);
        let mut result_buf: [u8; 1] = [0; 1];
        self.read_register(BMP3_REG_PWR_CTRL, &mut result_buf).await?;
        let mut pwr_ctrl: PWR_CTRL = PWR_CTRL(result_buf[0]);
        pwr_ctrl.set_power_mode(power_mode as u8);
        self.write_command([BMP3_REG_PWR_CTRL, pwr_ctrl.0]).await?;
        self.delayer.delay_ms(20).await;
        Ok(())
    }

    /// set air pressure enable state
    pub async fn set_pressure_enable(&mut self, enable: bool) -> Result<(), Error<E>> {
        debug!("in set_pressure_enable()");
        let mut result_buf: [u8; 1] = [0; 1];
        self.read_register(BMP3_REG_PWR_CTRL, & mut result_buf).await?;
        debug!("  read BMP3_REG_PWR_CTRL = {:b}", result_buf[0]);
        let mut pwr_ctrl: PWR_CTRL = PWR_CTRL(result_buf[0]);
        pwr_ctrl.set_press_en(enable);
        debug!("  writing pwr_ctrl {:#b}", pwr_ctrl.0);
        self.write_command([BMP3_REG_PWR_CTRL, pwr_ctrl.0]).await?;
        self.delayer.delay_ms(20).await;
        Ok(())
    }
    
    /// set temperature enable state
    pub async fn set_temperature_enable(&mut self, enable: bool) -> Result<(), Error<E>> {
        debug!("in set_temperature_enable()");
        let mut result_buf: [u8; 1] = [0; 1];
        self.read_register(BMP3_REG_PWR_CTRL, &mut result_buf).await?;
        debug!("  read BMP3_REG_PWR_CTRL = {:b}", result_buf[0]);
        let mut pwr_ctrl: PWR_CTRL = PWR_CTRL(result_buf[0]);
        debug!(" read PWR_CTRL reg = {:#b}", pwr_ctrl.0);
        pwr_ctrl.set_temp_en(enable);
        
        // line below should not be needed, but it is?
        //pwr_ctrl.set_power_mode(PowerMode::Normal as u8);


        debug!(" writing PWR_CTRL reg = {:#b} or {:?}", pwr_ctrl.0, pwr_ctrl);

        // pwr_ctrl.set_temp_en(false);
        // info!("  set_temp_en(false) = {:?}  ({:#b})", pwr_ctrl, pwr_ctrl.0);
        // pwr_ctrl.set_press_en(false);
        // info!("  set_press_en(false) = {:?}  ({:#b})", pwr_ctrl, pwr_ctrl.0);
        // pwr_ctrl.set_power_mode(PowerMode::Normal as u8);
        // info!("  set_power_mode(PowerMode::Normal) = {:?}  ({:#b})", pwr_ctrl, pwr_ctrl.0);


        self.write_command([BMP3_REG_PWR_CTRL, pwr_ctrl.0]).await?;
        self.delayer.delay_ms(20).await;
        Ok(())
    }

    /// set BMP38x configuration
    pub async fn set_bmp3_configuration(&mut self, config:  Bmp3Configuration) -> Result<(), Error<E>> {
        // not using the bitfield struct because it does not support const builder
        debug!("in set_bmp3_configuration({:?})", config);
        // set power mode
        self.set_power_control_mode(config.power_mode).await?;
        debug!("  set_power_control_mode to {:?}", config.power_mode);
        // set temperature ans pressure enable states
        self.set_temperature_enable(config.temperature_enable).await?;
        self.set_pressure_enable(config.pressure_enable).await?;
        // set oversampling
        let osp: u8 = config.over_sampling_press as u8;
        let ost: u8 = (config.over_sampling_temp as u8) << 3;
        let osr: u8 = ost | osp;
        debug!("  writing REG_OSR to {:#b}", osr);
        self.write_command([BMP3_REG_OSR, osr]).await?;
        self.delayer.delay_ms(20);
        // set output data rate
        let odr: u8 = config.output_data_rate as u8;
        debug!("  writing REG_ODR to {:#b}", odr);
        self.write_command([BMP3_REG_ODR, odr]).await?;
        self.delayer.delay_ms(20);
        // set IIR filter
        let iir_coef: u8 = (config.iir_filter_coef as u8) << 1;
        debug!("  writing REG_CONFIG (IIR) to {:#b}", iir_coef);
        self.write_command([BMP3_REG_CONFIG, iir_coef]).await?;
        self.delayer.delay_ms(20).await;
        Ok(())
    }

    /// get BMP38x configuration:  control(power mode, pressure enable and temperature enable), over sampling, ODR, filter, interrupt control.
    pub async fn get_bmp3_configuration(&mut self) -> Result<Bmp3Configuration, Error<E>> {
        debug!("in get_bmp3_configuration");
        let mut result_buf: [u8; 7] = [0; 7];
        // first result_buf is INT_CONF, then IF_CONF(skip), PWR_CTRL, OSR, ODR, skip byte, IIR_CONFIG
        self.read_register(BMP3_REG_INT_CTRL, &mut result_buf).await?;
        debug!(" read config = {:?}", result_buf);
        let _int_pin = result_buf[0];
        //info!(" int_pin = {:?}", int_pin);
        let pwr_ctrl: PWR_CTRL = PWR_CTRL(result_buf[2]); // power_mode bit are not set correctly
        debug!(" pwr_ctrl = {:?}", pwr_ctrl);
        let pwr_mode = self.get_power_mode().await?;  // must read PWR_CTRL register separately for this
        let temp_enable = pwr_ctrl.get_temp_en();
        let press_enable = pwr_ctrl.get_press_en();
        let osr_p = Over_Sampling::from(result_buf[3] & 0x07);
        //info!(" osr_p = {:?}", osr_p);
        let osr_t = Over_Sampling::from((result_buf[3] & 0x38) >> 3);
        //info!(" osr_t = {:?}", osr_t);
        let odr: Odr = Odr::from(result_buf[4] & 0x3f);
        //info!(" odr = {:?}", odr);
        let iir_filter = FilterCoef::from(result_buf[6]>>1);
        //info!(" iir_filter = {:?}", iir_filter);

        Ok(Bmp3Configuration {
            power_mode:  pwr_mode,
            temperature_enable: temp_enable,
            pressure_enable: press_enable,
            over_sampling_temp: osr_t,
            over_sampling_press: osr_p,
            output_data_rate: odr,
            iir_filter_coef: iir_filter,
        })

    }

        

    /// set interrupt pin configuration
    pub async fn set_interrupt_pin_config(&mut self, intpin_config: InterruptPinControl) -> Result<(), Error<E>> {
        debug!("in set_interrupt_pin_config()");
        let status = self.get_status().await?;
        if (status.get_cmd_ready()) {
            self.write_command([BMP3_REG_INT_CTRL, intpin_config.0] ).await?;
            self.delayer.delay_ms(20);
            Ok(())
        } else {
            Err(Error::NotReadyForCommand)
        }
    }

    /// get interrupt pin configuration
    pub async fn get_interrupt_pin_configuration(&mut self) -> Result<InterruptPinControl, Error<E>> {
        debug!("in get_interrupt_pin_configuration");
        let mut result_buf: [u8; 1] = [0; 1];
        self.read_register(BMP3_REG_INT_CTRL, &mut result_buf).await?;
        info!("  raw value is {:b}", result_buf[0]);
        Ok(InterruptPinControl(result_buf[0]))
    }


    /// get interrupt status INT_STATUS, register cleared after this called by BMP38x
    pub async fn get_interrupt_status(&mut self) -> Result<IntStatus, Error<E>> {
        debug!("in get_interrupt_status");
        let mut result_buf: [u8; 1] = [0; 1];
        self.read_register(BMP3_REG_INT_STATUS, &mut result_buf).await?;
        Ok(IntStatus(result_buf[0]))
    }
    
    /// read calibration registers and save into device property
    pub async fn read_calibration_pars(&mut self) -> Result<(), Error<E>> {
        let mut result_buf: [u8; 21] = [0; 21];
        self.read_register(BMP3_REG_CALIB_DATA+1, &mut result_buf).await;

        let nvm_par_t1:  u16 = result_buf[0] as u16 | ( (result_buf[1] as u16) << 8 );
        let nvm_par_t2:  u16 = result_buf[2] as u16 | ( (result_buf[3] as u16) << 8 );
        let nvm_par_t3:  i8 = result_buf[4] as i8;
        // convert to usable calibration pars
        self.cal_pars.par_t1_f = (nvm_par_t1 as f64) / 0.003_906_25;
        self.cal_pars.par_t2_f = (nvm_par_t2 as f64) / 1_073_741_824.0;
        self.cal_pars.par_t3_f = (nvm_par_t3 as f64) / 281_474_976_710_656.0;
 
        let nvm_par_p1:  i16  = ( (result_buf[6] as i16) << 8 ) | result_buf[5]  as i16 ;
        let nvm_par_p2:  i16  = ( (result_buf[8] as i16) << 8 ) | result_buf[7]  as i16 ;
        let nvm_par_p3:  i8  = result_buf[9]  as i8;
        let nvm_par_p4:  i8  = result_buf[10] as i8;
        let nvm_par_p5:  u16  = ( (result_buf[12] as u16) << 8 ) | result_buf[11] as u16 ;
        let nvm_par_p6:  u16  = ( (result_buf[14] as u16) << 8 ) | result_buf[13] as u16 ;


        let nvm_par_p7:  i8  = result_buf[15] as i8;
        let nvm_par_p8:  i8  = result_buf[16] as i8;
        let nvm_par_p9:  i16  = ( (result_buf[18] as i16) << 8 ) | result_buf[17] as i16 ;
        let nvm_par_p10: i8 = result_buf[19] as i8;
        let nvm_par_p11: i8 = result_buf[20] as i8;

        // info!("nvm_par_p1 = {}", nvm_par_p1);
        // info!("nvm_par_p2 = {}", nvm_par_p2);
        // info!("nvm_par_p3 = {}", nvm_par_p3);
        // info!("nvm_par_p4 = {}", nvm_par_p4);
        // info!("nvm_par_p5 = {}", nvm_par_p5);
        // info!("nvm_par_p6 = {}", nvm_par_p6);
        // info!("nvm_par_p7 = {}", nvm_par_p7);
        // info!("nvm_par_p8 = {}", nvm_par_p8);
        // info!("nvm_par_p9 = {}", nvm_par_p9);
        // info!("nvm_par_p10 = {}", nvm_par_p10);
        // info!("nvm_par_p11 = {}", nvm_par_p11);

        // convert to usable calibration pars
        self.cal_pars.par_p1_f =  (nvm_par_p1 as f64 - 16384.0) / 1048576.0;
        self.cal_pars.par_p2_f =  (nvm_par_p2 as f64 - 16384.0) / 536870912.0;
        self.cal_pars.par_p3_f =  (nvm_par_p3 as f64) / 4294967296.0;
        self.cal_pars.par_p4_f =  (nvm_par_p4 as f64) / 137438953472.0;
        self.cal_pars.par_p5_f =  (nvm_par_p5 as f64) / 0.125; 
        self.cal_pars.par_p6_f =  (nvm_par_p6 as f64) / 64.0;
        self.cal_pars.par_p7_f =  (nvm_par_p7 as f64) / 256.0;
        self.cal_pars.par_p8_f =  (nvm_par_p8 as f64) / 32768.0;
        self.cal_pars.par_p9_f =  (nvm_par_p9 as f64) / 281474976710656.0;
        self.cal_pars.par_p10_f = (nvm_par_p10 as f64) / 281474976710656.0;
        self.cal_pars.par_p11_f = (nvm_par_p11 as f64) / 36893488147419103232.0;
    
        Ok(())
    }
    
    pub fn compensate_temperature(&self,  raw_temperature: u32) -> f64 {      
        let temp_1 = (raw_temperature as f64) - self.cal_pars.par_t1_f;
        let temp_2 = (temp_1 as f64) * self.cal_pars.par_t2_f;
        let temperature = temp_2 + (temp_1 * temp_1) * self.cal_pars.par_t3_f;
        
        // temp_1 = temp_1 * self.cal_pars.par_t2_f;
        // temp_1 = (temp_1 * temp_1) * self.cal_pars.par_t3_f;
        temperature
    }
    
    pub fn compensate_pressure(&self, raw_pressure: f64, temperature: f64) -> f64 {
        debug!("in compensate_pressure({})", temperature);
        // roughly translated from Bosch manual's C code

        let calc_part1 = self.cal_pars.par_p6_f * temperature;
        let calc_part2 = self.cal_pars.par_p7_f * (temperature * temperature);
        let calc_part3 = self.cal_pars.par_p8_f * (temperature * temperature * temperature);
        let partial_1 = self.cal_pars.par_p5_f + calc_part1 + calc_part2 + calc_part3;
        debug!("  partial_1  _out1 = {}", partial_1);

        let calc_part1 = self.cal_pars.par_p2_f * temperature;
        let calc_part2 = self.cal_pars.par_p3_f * (temperature * temperature);
        
        let calc_part3 = self.cal_pars.par_p4_f * (temperature * temperature * temperature);
        let partial_2 = raw_pressure * ( self.cal_pars.par_p1_f + calc_part1 + calc_part2 + calc_part3);
        debug!("  partial_2   out2= {}", partial_2);
        
        let calc_part1 = raw_pressure * raw_pressure ;
        let calc_part2 = self.cal_pars.par_p9_f + self.cal_pars.par_p10_f * temperature;
        let calc_part3 = calc_part1 * calc_part2;
        
        let partial_3 = calc_part3 + (raw_pressure * raw_pressure * raw_pressure) * self.cal_pars.par_p11_f;
        debug!("  partial_3  partial_data4 = {}", partial_3);

        partial_1 + partial_2 + partial_3
      

        

    }
    /// get equivalent sealevel air pressure for given altitude (in metres)
    pub fn get_sealevel_airpressure(&self, air_pressure: f64, temperature: f64, altitude_m: f64) -> f64 {
         
        let exponent = -9.80665 * 0.028964 * altitude_m / (8.31432 * (temperature + 273.15));
        let base: f64 = 2.71828175f64;
        let air_press_at_sealevel = (air_pressure / 100.0) / pow(base, exponent) * 100.00;
         // kind-a silly to /100.0 and * 100.0 again, just more reabable
        air_press_at_sealevel
    }
    
    /// read the temperature and air pressure measurements with a specifica altitude of BMP38x device
    pub async fn read_measurements_with_altitude(&mut self, altitude: f64) -> Result<Measurements, Error<E>> {
        debug!("in read_measurements_with_altitude");
        let mut result_buf: [u8; 6] = [0; 6];
        let command_buf: [u8; 1] = [BMP3_REG_PRESS_DATA];
        // self.i2c
        //     .write_read(self.address, &command_buf, &mut result_buf).await
        //     .map_err(Error::I2c)?;
        self.read_register(BMP3_REG_PRESS_DATA, &mut result_buf).await?;
        let raw_pressure = result_buf[0] as u32 | ((result_buf[1] as u32) << 8) | (result_buf[2] as u32) << 16;
        let raw_temperature = result_buf[3] as u32 | ((result_buf[4] as u32) << 8) | (result_buf[5] as u32) << 16;
        debug!(" read_measurement_with..");
        debug!(" raw_pressure = {:?}", raw_pressure);
        //info!(" raw_temperature = {:?}", raw_temperature);
        
        let compensated_temp = self.compensate_temperature(raw_temperature);
        let compensated_press = self.compensate_pressure(raw_pressure as f64, compensated_temp as f64);
        
        let sealevel_press = self.get_sealevel_airpressure(compensated_press as f64, compensated_temp, altitude);
        self.altitude_m = altitude;

        Ok(Measurements { temperature_c: compensated_temp, air_pressure_pa: compensated_press , 
                air_pressure_sealevel_pa: sealevel_press })
        
    }

    /// read the temperature and air pressure measurements new ( 0 m) or new_with_altitude initialized altitude value 
    pub async fn read_measurements(&mut self) ->  Result<Measurements, Error<E>> {
        debug!("in read_measurements");
        let mut result_buf: [u8; 6] = [0; 6];
        let command_buf: [u8; 1] = [BMP3_REG_PRESS_DATA];
        self.i2c
            .write_read(self.address, &command_buf, &mut result_buf).await
            .map_err(Error::I2c)?;
        let raw_pressure = result_buf[0] as u32 | ((result_buf[1] as u32) << 8) | (result_buf[2] as u32) << 16;
        let raw_temperature = result_buf[3] as u32 | ((result_buf[4] as u32) << 8) | (result_buf[5] as u32) << 16;
        
        let compensated_temp = self.compensate_temperature(raw_temperature);
        let compensated_press = self.compensate_pressure(raw_pressure as f64, compensated_temp as f64);
  
        let sealevel_press = self.get_sealevel_airpressure(compensated_press, compensated_temp, self.altitude_m);

        Ok(Measurements { temperature_c: compensated_temp, air_pressure_pa: compensated_press , 
                air_pressure_sealevel_pa: sealevel_press })

     

    }

    /// get the power cotrol mode register value
    pub async fn get_power_mode(&mut self) -> Result<PowerMode, Error<E>> {
        debug!("in get_power_control_mode");
        let mut result_buf: [u8; 1] = [ 0; 1];
        self.read_register(BMP3_REG_PWR_CTRL, &mut result_buf).await?;
        debug!(" get_power_control_mode = {:#b}", result_buf[0] & 0b110000 >> 4);
        Ok(PowerMode::from(result_buf[0] & 0b110000 >> 4))
    }

    /// set power control mode
    pub async fn set_power_control_mode(&mut self, pwr_mode: PowerMode) -> Result<(), Error<E>> {
        debug!("in set_power_control_mode");
        let mut result_buf: [u8; 1] = [0; 1];
        let mut command_buf: [u8; 1] = [0; 1];
        // read the power control mode register first, it clears a flag there
        self.read_register(BMP3_REG_PWR_CTRL, &mut result_buf).await?;
        command_buf[0] = ((pwr_mode as u8) << 4) | (result_buf[0] & 0x03);  // preserve temp and press enable bits
        self.write_command( [BMP3_REG_PWR_CTRL, command_buf[0]] ).await?;
        self.delayer.delay_ms(10).await;
        debug!(" set_power_control_mode(), writing  PWR_CTRL reg = {:?} ({:#b})", pwr_mode, command_buf[0]);
        Ok(())


    }

    /// set FIFO configuration
    pub async fn set_fifo_config(&mut self, fifo_config: Fifo_Config) -> Result<(), Error<E>> {
        debug!("in set_fifo_config");
        self.write_command([BMP3_REG_FIFO_CONFIG_1, fifo_config.fifo_config_1.0] ).await?;
        self.delayer.delay_ms(5);
        self.write_command([BMP3_REG_FIFO_CONFIG_2, fifo_config.fifo_config_2.0] ).await?;
        self.fifo_config = fifo_config;

        Ok(())
    }

    /// get FIFO configuration
    pub async fn get_fifo_config(&mut self) -> Result<Fifo_Config, Error<E>> {
        debug!("in get_fifo_config");
        let mut result_buf: [u8; 2] = [0; 2];
        self.read_register(BMP3_REG_FIFO_CONFIG_1, &mut result_buf).await?;
        debug!("  result_buf[0] = {:#b}", result_buf[0]);
        debug!("  result_buf[1] = {:#b}", result_buf[1]);

        let config_1s: FifoConfig1 = FifoConfig1(result_buf[0]);  // FifoConfig1 does not impl Copy trait
        self.fifo_config.fifo_config_1 = config_1s;  // save in BMP38x properties
        let config_1: FifoConfig1 = FifoConfig1(result_buf[0]); 
        debug!("  config_1 = {:?}", config_1);

        self.delayer.delay_ms(5);
        let config_2s: FifoConfig2 = FifoConfig2(result_buf[1]);   // FifoConfi21 does not impl Copy trait
        self.fifo_config.fifo_config_2 = config_2s;  // save in BMP38x properties
        let config_2: FifoConfig2 = FifoConfig2(result_buf[1]);
        
        debug!("  config_2 = {:?}", config_2);
        let fifo_config: Fifo_Config = Fifo_Config { fifo_config_1: config_1, fifo_config_2: config_2 };
        
        Ok(fifo_config)
        
    }

    /// flush fifo buffer
    pub async fn fifo_flush(&mut self) -> Result<(), Error<E>> {
        debug!("in fifo_flush");
        // check for (new) command ready
        let mut result_buf: [u8; 1] = [0; 1];
        let status = self.get_status().await?;
        if (status.get_cmd_ready()) {
            //let mut write_buf: [u8; 2] = [BMP3_REG_CMD, BMP3_FIFO_FLUSH];
            self.write_command([BMP3_REG_CMD, BMP3_FIFO_FLUSH]).await?;

            // self.i2c.write(self.address, &write_buf).await
            //         .map_err(Error::I2c);

            //self.write_command([BMP3_REG_CMD, write_buf]).await? ; 
            self.delayer.delay_ms(5).await;
            self.read_register(BMP3_REG_ERR, &mut result_buf).await?;
            let error_value = ErrorReg(result_buf[0]);
            debug!("  write CMD returned {:x}", result_buf[0]);
            if ( error_value.get_cfg_error() || error_value.get_cmd_error() || error_value.get_fatal_error() ) {
                return Err(Error::CommandError);
            } 
            return Ok(());
        }
        Err(Error::NotReadyForCommand)   

    }

    /// get FIFO number of frames
    pub async fn get_fifo_number_frames(&mut self) -> Result<u16, Error<E>> {
        debug!("in get_fifo_number_frames");
        let mut result_buf: [u8; 2] = [ 0; 2];
        self.read_register(BMP3_REG_FIFO_LENGTH, &mut result_buf).await?;  // in bytes
        let fifo_length_bytes: u16 = u16::from_le_bytes(result_buf);
        debug!("  fifo_length_bytes = {}", fifo_length_bytes);
        // Determine the number of bytes per data frame.
        // 1 byte for header, 3 bytes for each sensor data if enabled
    
        let bytes_per_frame: u16 = 1 + 3 * (self.fifo_config.fifo_config_1.get_fifo_press_enable() as u16 + 
                                        self.fifo_config.fifo_config_1.get_fifo_temp_enable() as u16);

        // Set number of data frames
        Ok(fifo_length_bytes / bytes_per_frame)
    }
    
    /// get FIFO frame:  could be a control frame, sensor data frame or empty frame
    pub async fn get_fifo_frame(&mut self) -> Result<FifoFrame, Error<E>> {
        debug!("in get_fifo_frame");
        let mut result_buf: [u8; 1] = [0; 1];
        self.read_register(BMP3_REG_FIFO_DATA, &mut result_buf).await?;  // read Fifo header
        //  MSB 7,6 = 0b10 sensor frame, check bits s,t and p (5,4 and 2)
        //          = 0b01 Control frame
        //              then if bits 5,4,3,2 = 0b0001 = Control frame config error
        //              or if bits 5,4,3,2 = 0b0010 = Control frame config changed
        //      1,0 = 0b00 always
        let fh = result_buf[0];
        debug!("  header = {:b}", fh);
        if (fh & 0b1000_0000 != 0x00)  {
            // sensor data frame:  could be data or time frame
            // was if (fh & 0b10100000 != 0x00)  {  // a time frame
            if (fh & 0x20 == 0x20)  {  // a time frame
                debug!("  a time frame");
                let mut result_buf: [u8; 4] = [0; 4];
                self.read_register(BMP3_REG_FIFO_DATA, &mut result_buf).await?;
                let time_stamp: u32 = u32::from_le_bytes([result_buf[1], result_buf[2], result_buf[3], 0]);
                return Ok( FifoFrame::SensorDataFrame(SensorFrameEnum::Time(time_stamp)) )  

            } else if (fh & 0x14 == 0x14) {  // a temperature and pressure frame, check for this first otherwise result is just 
                                             // temp or press
                debug!("  a temperature and pressure frame");
                let mut result_buf: [u8; 7] = [0; 7];
                self.read_register(BMP3_REG_FIFO_DATA, &mut result_buf).await?;
                let raw_temperature = result_buf[1] as u32 | ((result_buf[2] as u32) << 8) | (result_buf[3] as u32) << 16;
                let compensated_temp = self.compensate_temperature(raw_temperature);
                let raw_pressure: u32 = result_buf[4] as u32 | ((result_buf[5] as u32) << 8) | (result_buf[6] as u32) << 16;
                let compensated_press = self.compensate_pressure(raw_pressure as f64, compensated_temp);
                let tandP: TandPtype = TandPtype{temperature: compensated_temp, pressure: compensated_press };
                return Ok( FifoFrame::SensorDataFrame(SensorFrameEnum::TemperatureAndPressureData(tandP)) )    

            } else if (fh & 0x10 == 0x10)  {  // a temperature frame
                debug!("  a temperature frame");
                let mut result_buf: [u8; 4] = [0; 4];
                self.read_register(BMP3_REG_FIFO_DATA, &mut result_buf).await?;
                let raw_temperature = result_buf[1] as u32 | ((result_buf[2] as u32) << 8) | (result_buf[3] as u32) << 16;
                let compensated_temp = self.compensate_temperature(raw_temperature);
                return Ok( FifoFrame::SensorDataFrame(SensorFrameEnum::TemperatureData(compensated_temp)) )

            } else if (fh & 0x04 == 0x04) { // a pressure frame
                debug!("  a pressure frame");
                let mut result_buf: [u8; 4] = [0; 4];
                self.read_register(BMP3_REG_FIFO_DATA, &mut result_buf).await?;
                let raw_pressure: u32 = result_buf[1] as u32 | ((result_buf[2] as u32) << 8) | (result_buf[3] as u32) << 16;

                let compensated_press = self.compensate_pressure(raw_pressure as f64, 0.0);  // set unknown temp to 0 for compensation
                return Ok( FifoFrame::SensorDataFrame(SensorFrameEnum::PressureData(compensated_press)) )

  
            } else {
                debug!("  an empty frame");
                return Ok(FifoFrame::SensorDataFrame(SensorFrameEnum::Empty))
            }


        }
        else if (fh & 0b01000000 != 0x00) {
            // control frame
            debug!("  a control frame");
            let mut result_buf: [u8; 2] = [ 0; 2];
            self.read_register(BMP3_REG_FIFO_DATA, &mut result_buf).await?;  // read but ignore result_buf
            if (fh & 0b01000100 == 0x44) { 
                // control frame fifo config error
                debug!("   a configuration error");
                return Ok(FifoFrame::ControlFrame(ControlFrameEnum::ConfigurationError))
                
            } else if (fh & 0b01001000 == 0x48) {
                debug!("   a configuration change");
                return Ok(FifoFrame::ControlFrame(ControlFrameEnum::ConfigurationChange))
            } else {
                debug!("   an unknown control frame, header =  {:x}", fh);
                return Err(Error::FifoUnknownContent)
            }
        } else {
                return Err(Error::FifoUnknownContent)
        }
        

        // 

    }

    /*
    
    //---------------------------------
    

    /// read relative humidity in percent
    pub async fn read_relative_humidity(&mut self) -> Result<f32, Error<E>> {
        debug!("in read_relative_humidity()");
        self.delayer.delay_ms(20).await;  // in case user called this again too quickly
        let command_buffer: [u8; 1] = [Si7021_READ_RH_NO_HOLD]; 
        self.i2c.write(self.address, &command_buffer).await
            .map_err(Error::I2c)?;
        self.delayer.delay_ms(25).await;
        debug!("did i2c write, next is read");
        let mut result_buf: [u8; 3] = [0; 3];
        self.i2c.read(self.address, &mut result_buf).await
            .map_err(Error::I2c)?;
        
        let humidity_u16 : u16 = u16::from_be_bytes( [result_buf[0], result_buf[1] ]);
        // scale it
        let mut humidity: f32 = ( humidity_u16 as f32 * 125.0 / 65536.0 ) - 6.0;
        
        // clamp value to => 0 and <= 100
        #[allow(unused_parens)]
        if (humidity.is_sign_negative() ) {
            humidity = 0.0;
        }
        #[allow(unused_parens)]
        if (humidity > 100.0) {
            humidity = 100.0
        }

        Ok(humidity)
        

    }


    /// read temperatue in degrees C
    pub async fn read_temperature(&mut self) -> Result<f32, Error<E>> {
        debug!("in read_temperature()");
        let command_buffer: [u8; 1] = [Si7021_READ_TEMP_NO_HOLD]; 
        self.i2c.write(self.address, &command_buffer).await
                .map_err(Error::I2c)?;
        self.delayer.delay_ms(20).await;
        debug!("did i2c write, next is read");
        let mut result_buf: [u8; 3] = [0; 3];
        self.i2c.read(self.address, &mut result_buf).await
                .map_err(Error::I2c)?;
        
        let temperature_u16 : u16 = u16::from_be_bytes( [result_buf[0], result_buf[1] ]);
        // scale it
        let temperature: f32 = ( temperature_u16 as f32 * 175.72 / 65536.0 ) - 46.85;

        Ok(temperature)
    }

    /// read measurements (temperature and humidity as a struct)
    pub async fn read_measurements(&mut self) -> Result<Measurements, Error<E>> {
        debug!("in read_measurements()");
        let humidity = self.read_relative_humidity().await?;
        //self.delayer.delay_ms(5);
        let mut result_buf: [u8; 2] = [0; 2];
        self.read_register(Si7021_READ_TEMP_AFTER_PREVIOUS_RH, &mut result_buf).await?;
        let temperature_u16 : u16 = u16::from_be_bytes( [result_buf[0], result_buf[1] ]);
        // scale it
        let temperature: f32 = ( temperature_u16 as f32 * 175.72 / 65536.0 ) - 46.85;

        let measurements: Measurements = Measurements {
            relative_humidity_percent: humidity,
            temperature_c: temperature,
        };
        Ok(measurements)
    }  
  
    /// heater control enable/disable
    pub async fn heater_control(&mut self, enable: bool) -> Result<(), Error<E>> {
        debug!("in heater_control({})", enable);
        let mut result_buf: [u8; 1] = [0; 1];
        self.read_register(Si7021_READ_RH_T_USER_REG_1, &mut result_buf).await?;
        let mut write_value = result_buf[0];
        if enable {
            write_value = write_value | (1 << 2);
        } else {
            write_value = write_value & !(1 << 2);
        }
        self.write_command([Si7021_WRITE_RH_T_USER_REG_1, write_value]).await?;

        Ok(())
    }

    /// is heater enabled
    pub async fn is_heater_enabled(&mut self) -> Result<bool, Error<E>> {
        debug!("in is_heater_enabled()");
        let mut result_buf: [u8; 1] = [0; 1];
        self.read_register(Si7021_READ_RH_T_USER_REG_1, &mut result_buf).await?;
        let read_value = result_buf[0];
        #[allow(unused_parens)]
        if ((read_value & 0x04) != 0x00)  {
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// set heater power level
    pub async fn set_heater_level(&mut self, level: u8) -> Result<(), Error<E>> {
        debug!("in set_heater_level ( {} )", level);
        #[allow(unused_parens)]
        if (level > 0x0f) {
            return Err(Error::OutOfRange(level));
        }
        self.write_command([Si7021_WRITE_HEATER_CONTROL, level]).await?;
        Ok(())
    }

    */

}
