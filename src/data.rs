use core::default;

// no_std support
#[allow(unused_imports)]
#[allow(dead_code)]
// use libm::{exp, round, trunc};
use log::debug;
use bitfield::bitfield;
use const_builder::ConstBuilder;


#[allow(unused_imports)] // for no_std use


/// A measurement result from the sensor.
#[derive(Debug, PartialEq, Clone, Copy)]
pub struct Measurements {
    /// temperature degrees C
    pub temperature_c: f64,
    /// relative humidity percent
    pub air_pressure_pa: f64,
    /// equivalent setlevel pressure
    pub air_pressure_sealevel_pa: f64,
}

/// Si7021 device id
#[derive(Debug, PartialEq, Clone, Copy)]
pub struct DeviceInfo {
    /// device ID
    pub id: DeviceModel, 
}

impl Default for DeviceInfo {
    fn default() -> Self {
        DeviceInfo {
            id: DeviceModel::NotRead,
        }
    }
}

/// interrupt pin active state
#[derive(Debug, PartialEq, Clone, Copy)]
#[repr(u8)]
pub enum IntActiveState {
    LOW = 0x00,
    HIGH = 0x01,
}

impl From<u8> for IntActiveState {
    fn from(v: u8) -> Self {
        match v {
            0x00 => Self::LOW,
            0x01 => Self::HIGH,
            _   => unreachable!(),
        }
    }
}

/// interrupt pin mode
#[derive(Debug, PartialEq, Clone, Copy)]
#[allow(non_camel_case_types)]
#[repr(u8)]
pub enum IntOutputMode {
    PUSH_PULL = 0x00,
    OPEN_DRAIN = 0x01,
}

impl From<u8> for IntOutputMode {
    fn from(v: u8) -> Self {
        match v {
            0x00 => Self::PUSH_PULL,
            0x01 => Self::OPEN_DRAIN,
            _   => unreachable!(),
        }
    }
}

/// device model, BMP 384, 388 and 390 supported (390 not tested but should work, same API)
#[derive(Debug, PartialEq, Clone, Copy)]
#[repr(u8)]
pub enum DeviceModel {
    EngineeringSample = 0xff,
    BMP384_8 = 0x50,
    BMP390 = 0x60,
    NotRead = 0x00,
}

impl From<u8> for DeviceModel {
    fn from(v: u8) -> Self {
        match v {
            0xff => Self::EngineeringSample,
            0x50 => Self::BMP384_8,
            0x60 => Self::BMP390,
            0x00 => Self::NotRead,
            _ => unreachable!(),
        }
    }
}

/// Power Mode
#[derive(Debug, Eq, PartialOrd, PartialEq, Clone, Copy, Default)]
#[repr(u8)]
pub enum PowerMode {
    Sleep =  0x00,
    Forced = 0x01,
    #[default]
    Normal = 0x03,
}

impl From<u8> for PowerMode {
    fn from(v: u8) -> PowerMode {
        match v {
            0x00 => Self::Sleep,
            0x01 => Self::Forced,
            0x02 => Self::Forced,
            0x03 => Self::Normal,
            _    => unreachable!(),
        }
    }
}



bitfield! {
    /// BMP38x PWR_CTRL register
    pub struct PWR_CTRL(u8);
    impl Debug;

    pub  get_press_en, set_press_en: 0;  // pressure enable = 1, disable = 0
    pub  get_temp_en,  set_temp_en:   1;  // temperature enable = 1, disable = 0
    pub into PowerMode, get_power_mode, set_power_mode: 5,4;  // power mode bits

}


bitfield! {
    /// BMP38x STATUS bits
    // BMP38x STATUS bits
    pub struct Status(u8);
    impl Debug;

    pub bool, get_temp_ready, _: 6;    // Data ready for temperature sensor
    pub bool, get_press_ready, _: 5;             // Data ready for pressure.
    pub bool, get_cmd_ready, _: 4;   // CMD decoder status, ready for new command when == 1
    // bits 7,3,2,1,0 not used
}

bitfield! {
    /// BMP38x INT_STATUS bits
    // BMP38x INT_STATUS bits
    pub struct IntStatus(u8);
    impl Debug;

    pub bool, get_fifo_watermark, _: 0;    // FIFO watermark interrupt
    pub bool, get_fifo_full, _: 1;         // FIFO full interrupt
    pub bool, get_data_ready, _: 3;        // data ready interrupt
    // bits 7,6,5,4,2 not used
}

bitfield! {
    /// BMP38x Error bits
    pub struct ErrorReg(u8);
    impl Debug;
    
    pub bool, get_fatal_error, set_fatal_error: 0;  // fatal error
    pub bool, get_cmd_error, set_cmd_error: 1;  // Command execution failed. Cleared on read
    pub bool, get_cfg_error, set_cfg_error: 2;  // sensor configuration error detected. Cleared on read
    // bits 7 - 3 not used
}




/// Output Data Rate (ODR) config
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Default)]
#[allow(non_camel_case_types)]
#[repr(u8)]
pub enum Odr {
    ODR_200 =   0x00,  /// 200 Hz or 5 ms period
    ODR_100 =   0x01,  /// 100 Hz or 10 ms
    #[default]
    ODR_50  =   0x02,  /// 50 Hz
    ODR_25  =   0x03,  /// 25 Hz
    ODR_12P5 =  0x04, /// 25/2 Hz or 80 ms
    ODR_6P25 =  0x05, /// 25/4 Hz or 160 ms
    ODR_3P1  =  0x06, /// 25/8 Hz or 320 ms
    ODR_3P5  =  0x07, /// 25/16 Hz or 640 ms
    ODR_0P78 =  0x08, /// 25/32 Hz or 1.280 s
    ODR_0P39 =  0x09, /// 25/64 Hz or 2.560 s
    ODR_0P2  =  0x0a, /// 25/128 Hz or 5.120 s
    ODR_0P1  =  0x0b, /// 25/256 Hz or 10.24 s
    ODR_0P05 =  0x0c, /// 25/512 Hz or 20.48 s
    ODR_0P02 =  0x0d, /// 25/1024 Hz or 40.96 s
    ODR_0P01 =  0x0e, /// 25/2048 Hz or 81.92 s
    ODR_0P006 = 0x0f, /// 25/4096 Hz or 163.84 s
    ODR_0P003 = 0x10, /// 25/8192 Hz or 327.68 s
    ODR_0P0015 = 0x11, // 25/16384 Hz or 655.36 s
}

impl From<u8> for Odr {
    fn from(v: u8) -> Self {
        match v {
          0x00 => Self::ODR_200,
          0x01 => Self::ODR_100,
          0x02 => Self::ODR_50,
          0x03 => Self::ODR_25,
          0x04 => Self::ODR_12P5,
          0x05 => Self::ODR_6P25,
          0x06 => Self::ODR_3P1,
          0x07 => Self::ODR_3P5,
          0x08 => Self::ODR_0P78,
          0x09 => Self::ODR_0P39,
          0x0a => Self::ODR_0P2,
          0x0b => Self::ODR_0P1,
          0x0c => Self::ODR_0P05,
          0x0d => Self::ODR_0P02,
          0x0e => Self::ODR_0P01,
          0x0f => Self::ODR_0P006,
          0x10 => Self::ODR_0P003,
          0x11 => Self::ODR_0P0015,
          _ => Self::ODR_200,  // default
        }
    }
}




/// Over Sampling  config
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Default)]
#[allow(non_camel_case_types)]
#[repr(u8)]
pub enum Over_Sampling {
    #[default]
    ULTRA_LOW_POWER       = 0x00,  /// Ultra low power x 1, 16 bit
    LOW_POWER             = 0x01,  /// Lower power x 2, 17 bit
    STANDARD_RESOLUTION   = 0x02,  /// Standard resultion x 4, 18 bit
    HIGH_RESOLUTION       = 0x03,  /// High resolution x 8, 19 bit
    ULTRA_HIGH_RESOLUTION = 0x04,  /// Ultra high resolution  x 16, 20 bit
    HIGHEST_RESOLUTION    = 0x05,  // Highest resultion x 32, 21 bit
}

impl From<u8> for Over_Sampling {
    fn from(v: u8) -> Self {
        match (v) {
            0x00 => Self::ULTRA_LOW_POWER,
            0x01 => Self::LOW_POWER,
            0x02 => Self::STANDARD_RESOLUTION,
            0x03 => Self::HIGH_RESOLUTION,
            0x04 => Self::ULTRA_HIGH_RESOLUTION,
            0x05 => Self::HIGHEST_RESOLUTION,
            _ => Self::STANDARD_RESOLUTION,
        }
    }
}

bitfield! {
    /// OSR : pressure and temperature oversampling
    pub struct OSR(u8);
    impl Debug;

    pub into Over_Sampling, get_press_os, set_press_os: 2, 0;  // was 2,3;  3 bits  pressure
    pub into Over_Sampling, get_temp_os,  set_temp_os:  5, 3;  // was 5,3; 3 bits  temperature
}

impl Default for OSR {
    fn default() -> Self {
        let osr: OSR = OSR(0x44);  // standard over sampling rates *4 or 18 bits for both pressure and temperature
        osr
    }
}


bitfield! {
    /// Interrupt pin configuration
    /// output_mode 0 (false) == push-pull, 1 (true) == open_drain
    /// active_level  0 (false) == active low, 1 (true) = active high
    /// the rest disabled, 1 == enabled
    pub struct InterruptPinControl(u8);
    impl Debug;

    pub bool, get_output_mode, set_output_mode: 0;      // 0 == push-pull, 1 == open_drain
    pub bool, get_active_level, set_active_state: 1;    // 0 == active low, 1 = active high
    pub bool, get_latching, set_latching: 2;            // 0 == disabled, 1 == enabled
    pub bool, get_fifo_wm, set_fifo_wm: 3;              // 0 == disabled, 1 == enabled
    pub bool, get_fifo_full, set_fifo_full: 4;          // 0 == disabled, 1 == enabled
    pub bool, get_temp_press_dr, set_temp_press_rd: 6;  // temper | press data_ready 0 == disabled, 1 == enabled

}

bitfield! {
    /// FIFO config 1
    pub struct FifoConfig1(u8);
    impl Debug;

    pub bool, get_fifo_enable, set_fifo_enable: 0;  // 0 (false = disabled), 1 (true) = enabled
    pub bool, get_fifo_stop_full,  set_fifo_stop_full: 1; // 0 = do not stop writing when full, 1 = stop when full
    pub bool, get_fifo_time_enable, set_fifo_time_enable: 2; // 0 = time not returned, 1 = return time after last data frame
    pub bool, get_fifo_press_enable, set_fifo_press_enable: 3; // 0 = diable pressure data in fifo, 1 = enable pressure data
    pub bool, get_fifo_temp_enable, set_fifo_temp_enable: 4; // 0 = disable temperature data in fifo, 1 = enable temperature data

}

/// FifoSubSampling
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Default)]
#[allow(non_camel_case_types)]
#[repr(u8)]
pub enum FifoSubSampling {
    #[default]
    ONE       = 0x00,       
    TWO       = 0x01,       
    FOUR      = 0x02,  
    EIGHT     = 0x03, 
    SIXTEEN   = 0x04,  
    THIRTYTWO = 0x05,
    SIXTYFOUR = 0x06,
    ONETWENTYEIGHT = 0x07,

}

impl From<u8> for FifoSubSampling {
    fn from(v: u8) -> Self {
        match (v) {
            0x00 => Self::ONE,
            0x01 => Self::TWO,
            0x02 => Self::FOUR,
            0x03 => Self::EIGHT,
            0x04 => Self::SIXTEEN,
            0x05 => Self::THIRTYTWO,
            0x06 => Self::SIXTYFOUR,
            0x07 => Self::ONETWENTYEIGHT,
            _ => Self::ONE,
        }
    }
}


/// FifoDataSelect
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Default)]
#[allow(non_camel_case_types)]
#[repr(u8)]
pub enum FifoDataSelect {
    #[default]
    UNFILTERED = 0x00,
    FILTERED =   0x01,
}

impl From<u8> for FifoDataSelect {
    fn from(v: u8) -> Self {
        match (v) {
            0x00 => Self::UNFILTERED,
            0x01 => Self::FILTERED,
            0x02 => Self::UNFILTERED,
            0x03 => Self::UNFILTERED,
            _    => unreachable!(),
        }
    }
}


bitfield! {
    /// FIFO config 2
    pub struct FifoConfig2(u8);
    impl Debug;

    pub into FifoSubSampling, get_fifo_subsampling, set_fifo_subsampling: 2, 0;
    pub into FifoDataSelect, get_fifo_data_select, set_fifo_data_select: 4, 3;
}

/// FIFO configuration 
#[derive(Debug)]
pub struct Fifo_Config {
    pub fifo_config_1:  FifoConfig1,
    pub fifo_config_2:  FifoConfig2,
}

impl Default for Fifo_Config {
    fn default() -> Self {
        Fifo_Config {
            fifo_config_1: FifoConfig1(0x00),
            fifo_config_2: FifoConfig2(0x00),
        }
    }
}


#[derive(Debug, PartialEq, Copy, Clone)]
pub enum ControlFrameEnum {
    ConfigurationError,
    ConfigurationChange,
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub struct TandPtype {
    pub temperature: f64,
    pub pressure:  f64,
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum SensorFrameEnum {
    Empty,
    PressureData(f64),
    TemperatureData(f64),
    TemperatureAndPressureData(TandPtype),
    Time(u32),
}

/// Fifo Frame: could be ControlFrame enum or SensorFrame enum
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum FifoFrame {
    ControlFrame(ControlFrameEnum),
    SensorDataFrame(SensorFrameEnum),
}

/// IIR Filter coefficients
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Default)]
#[repr(u8)]
pub enum FilterCoef {
    COEF_0   = 0x00,
    COEF_1   = 0x01,
    COEF_3   = 0x02,
    COEF_7   = 0x03,
    #[default]
    COEF_15  = 0x04,
    COEF_31  = 0x05,
    COEF_63  = 0x06,
    COEF_127 = 0x07,
}

impl From<u8> for FilterCoef {
    fn from(v: u8) -> Self {
        match (v) {
            0x00 => Self::COEF_0,
            0x01 => Self::COEF_1,
            0x02 => Self::COEF_3,
            0x03 => Self::COEF_7,
            0x04 => Self::COEF_15,
            0x05 => Self::COEF_31,
            0x06 => Self::COEF_63,
            0x07 => Self::COEF_127,
            _    => unreachable!(),
        }
    }
}

/// BMP3xx configuration for PowerMode, temp/press enable, oversampling, output data rate, IIR filter
///                      note that this struct uses a const builder so it cannot include the interrupt pin configs
#[derive(ConstBuilder,Debug, Clone, Copy, PartialEq, Eq, PartialOrd)]
pub struct Bmp3Configuration {
    #[builder(default = PowerMode::Normal)]
    pub power_mode:  PowerMode,
    #[builder(default = true)]
    pub temperature_enable: bool,
    #[builder(default = true)]
    pub pressure_enable: bool,
    #[builder(default = Over_Sampling::STANDARD_RESOLUTION)]
    pub over_sampling_temp: Over_Sampling,
    #[builder(default = Over_Sampling::HIGH_RESOLUTION)]
    pub over_sampling_press: Over_Sampling,
    #[builder(default = Odr::ODR_50)]
    pub output_data_rate:  Odr,
    #[builder(default = FilterCoef::COEF_15)]
    pub iir_filter_coef:  FilterCoef,
}

impl Default for Bmp3Configuration {
    fn default() -> Self {
        Self { 
            power_mode: Default::default(),
            temperature_enable:  true,
            pressure_enable: true,
            over_sampling_temp: Over_Sampling::STANDARD_RESOLUTION,
            over_sampling_press: Over_Sampling::HIGH_RESOLUTION,
            output_data_rate: Odr::default(),
            iir_filter_coef: FilterCoef::default(),
         }
    }
}



/// calibration parameters
#[derive(Debug, PartialEq, Clone, Copy, Default)]
pub struct CalibrationPars {
    // register values in bmp3x NV memory

    
    // usable parameters as f64 when compensating
    pub par_t1_f: f64,
    pub par_t2_f: f64,
    pub par_t3_f: f64,
    
    pub par_p1_f:  f64,
    pub par_p2_f:  f64,
    pub par_p3_f:  f64,
    pub par_p4_f:  f64,
    pub par_p5_f:  f64,
    pub par_p6_f:  f64,
    pub par_p7_f:  f64,
    pub par_p8_f:  f64,
    pub par_p9_f:  f64,
    pub par_p10_f: f64,
    pub par_p11_f: f64,
}


