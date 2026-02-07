// BMP384-388 registers

#![allow(nonstandard_style)]
pub const BMP3_REG_CHIP_ID: u8 = 0x00;  // Chip ID , expect value 0x50 
pub const BMP3_REG_ERR: u8 = 0x02;  // Sensor Error conditions, ToDo:  a bit field
pub const BMP3_REG_SENS_STATUS: u8 = 0x03;  // Sensor Status Flags, ToDo: a bit field
pub const BMP3_REG_PRESS_DATA: u8 = 0x04; // 3 bytes at 0x04 - 0x06, 24Bit pressure data, little endian
pub const BMP3_REG_TEMP_DATA: u8 = 0x07; // 3 bytes at 0x07 - 0x09, 24Bit temperature data, little endian
pub const BMP3_REG_SENSOR_TIME: u8 = 0x0c; // 3 bytes at 0x0c - 0x0e, 24Bit sensor time, little endian
pub const BMP3_REG_EVENT: u8 = 0x10; // “EVENT” register
pub const BMP3_REG_INT_STATUS: u8 = 0x11;  // “INT_STATUS” register shows interrupt status; ToDo: a bit field
pub const BMP3_REG_FIFO_LENGTH  : u8 = 0x12;  // 2 bytes at 00x12, 0x013 FIFO byte counter indicates the current fill level of the
                                                            // FIFO buffer. Its size is 9 bit for 512 bytes
pub const BMP3_REG_FIFO_DATA  : u8 = 0x14;  // “FIFO_DATA” is the data output register
pub const BMP3_REG_FIFO_WM: u8 = 0x15;  // 2 bytes at 0x15, 0x016FIFO Watermark size is 9 Bit
pub const BMP3_REG_FIFO_CONFIG_1: u8 = 0x17;  // “FIFO_CONFIG_1” register, ToDo: a bit field
pub const BMP3_REG_FIFO_CONFIG_2: u8 = 0x18;  // “FIFO_CONFIG_2” register extends the FIFO_CONFIG_1 register. ToDo: a bitfield
pub const BMP3_REG_INT_CTRL: u8 = 0x19;  // Interrupt configuration, ToDo: a bitfield
pub const BMP3_REG_IF_CONF: u8 = 0x1a;  // “IF_CONF” register controls the serial interface settings, ToDO: a bitfield
pub const BMP3_REG_PWR_CTRL: u8 = 0x1b;  // “PWR_CTRL” register, enables or disables pressure and temperature measuremen
                                            // ToDo: a bitfield
pub const BMP3_REG_OSR: u8 = 0x1c;  // “OSR” register controls the oversampling settings, ToDo: a bitfield ?
pub const BMP3_REG_ODR: u8 = 0x1d;  // “ODR” register set the configuration of the output data rates
pub const BMP3_REG_CONFIG: u8 = 0x1f;  // “CONFIG” register controls the IIR filter coefficient
pub const BMP3_REG_CALIB_DATA: u8 = 0x30;  // calibration data 
pub const BMP3_REG_CMD: u8 = 0x7e; // commands
pub const BMP3_SOFT_RESET: u8 = 0xb6; // yap, a software initiated device reset
pub const BMP3_FIFO_FLUSH: u8 = 0xb0;  // flush the fifo buffer


#[repr(u8)]
/// BMP38x I2C device address
#[derive(Debug, Clone, Copy)]
pub enum DeviceAddress {
    /// it only has one I2C address but let's call it primary in case future secondary is needed
    Primary = 0x76,
    Secondary = 0x77,
}

impl From<DeviceAddress> for u8 {
    fn from(value: DeviceAddress) -> Self {
        match value {
            DeviceAddress::Primary => 0x76,
            DeviceAddress::Secondary => 0x77,
        }
    }
}

impl Default for DeviceAddress {
    fn default() -> Self {
        Self::Primary
    }
}
