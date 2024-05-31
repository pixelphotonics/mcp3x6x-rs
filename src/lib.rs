//! Platform-agnostic driver for the Microchip
//! MCP3561, MCP3562, MCP3564, and MCP3561R, MCP3562R, MCP3564R 24-bit Delta-Sigma ADCs.
//! 
//! Implementation is based on the datasheet of the MCP356xR (which includes a voltage
//! reference), but should also work with the older MCP356x chips.
//! 
//! The devices support SPI modes 0 and 3.

#![cfg_attr(not(test), no_std)]

use config::{Config, IRQMode, Register, RegisterMux};
use embedded_hal::spi::SpiDevice;

pub mod config;

/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<E> {
    /// SPI bus error
    SPIError(E),

    /// Voltage is too high to be measured.
    VoltageTooHigh,

    /// Voltage is too low to be measured.
    VoltageTooLow,

    InvalidMuxSelection,

    /// When performing a single-shot conversion
    WrongConfig,

    /// Trying to write to a read-only register
    RegisterReadOnly,

    /// The IRQ_MODE register is not set to MDAT_OUT_IRQ_HIGH or MDAT_OUT_IRQ_Z, therefore
    /// no ADC data can be read from the ADCDATA register (since the MDAT output codes are stored there)
    MdatOutSelected,

    /// The data is not ready to be read.
    DataNotReady,

    /// Could not parse the configuration from the device.
    ConfigReadError,

    /// The device is locked and cannot be written to. Call `unlock()` first.
    DeviceLocked,
}


/// The hard-coded device address. The factory default is 0b01,
/// but other addresses are available upon request from Microchip.
pub enum DeviceAddress {
    Adr00,
    Adr01,
    Adr10,
    Adr11,
}

impl Default for DeviceAddress {
    fn default() -> Self {
        DeviceAddress::Adr01
    }
}

impl DeviceAddress {
    fn cmd_byte(&self) -> u8 {
        match self {
            DeviceAddress::Adr00 => 0b00 << 6,
            DeviceAddress::Adr01 => 0b01 << 6,
            DeviceAddress::Adr10 => 0b10 << 6,
            DeviceAddress::Adr11 => 0b11 << 6,
        }
    }
}

enum Command {
    Conversion,
    Standby,
    Shutdown,
    FullShutdown,
    FullReset,
    StaticRead,
    IncrementalWrite,
    IncrementalRead,
}

impl Command {
    fn to_byte(&self) -> u8 {
        match self {
            Command::Conversion => 0b101000,
            Command::Standby => 0b101100,
            Command::Shutdown => 0b110000,
            Command::FullShutdown => 0b110100,
            Command::FullReset => 0b111000,
            Command::StaticRead => 0b01,
            Command::IncrementalWrite => 0b11,
            Command::IncrementalRead => 0b10,
        }
    }
}

pub struct StatusByte(u8);

impl StatusByte {
    pub fn is_data_ready(&self) -> bool {
        self.0 & 0b0000_0100 == 0
    }

    pub fn is_crc_error(&self) -> bool {
        self.0 & 0b0000_0010 == 0
    }

    /// The POR bit is set to 0 after a power-on reset, and 1 for each subsequent read.
    pub fn por(&self) -> bool {
        self.0 & 0b0000_0001 == 0
    }
}


/// Convert the raw ADC data to an i32, depending on the configured data format.
fn convert_adc_data(raw_data: &[u8], data_format: config::DataFormat) -> i32 {
    match data_format {
        config::DataFormat::Size32Data25WithId => {
            let _channel_id = raw_data[0] & 0b1111_0000;
            let sign_byte = (raw_data[0] & 0b0000_1111) | ((raw_data[0] & 0b0000_1111) << 4);   // We have a 4-bit sign extension, and we extend it to 8 bits
            let raw_data = [sign_byte, raw_data[1], raw_data[2], raw_data[3]];
            i32::from_be_bytes(raw_data)
        },
        config::DataFormat::Size32Data25 => {
            // This should already be right-aligned
            i32::from_be_bytes([raw_data[0], raw_data[1], raw_data[2], raw_data[3]])
        },
        config::DataFormat::Size32Data24Left => {
            let raw_data = [raw_data[0], raw_data[1], raw_data[2], raw_data[3]];
            i32::from_be_bytes(raw_data) >> 8
        },
        config::DataFormat::Size24 => {
            let raw_data = [raw_data[0], raw_data[1], raw_data[2], 0];
            i32::from_be_bytes(raw_data) >> 8
            // Since we have an i32, this adds a sign extension (arithmetic shift)
            // ("Arithmetic right shift on signed integer types, logical right shift on unsigned integer types.",
            //  source: https://doc.rust-lang.org/reference/expressions/operator-expr.html#arithmetic-and-logical-binary-operators)
        },
    }
}




pub struct MCP356x<SPI: SpiDevice> {
    spi: SPI,
    addr: DeviceAddress,
    config: config::Config,
}

impl<SPI: SpiDevice> MCP356x<SPI> {
    pub fn new(spi: SPI) -> Self {
        MCP356x {
            spi,
            addr: DeviceAddress::default(),
            config: config::Config::default(),
        }
    }

    pub fn new_with_address(spi: SPI, addr: DeviceAddress) -> Self {
        MCP356x {
            spi,
            addr,
            config: config::Config::default(),
        }
    }

    fn fast_command(&mut self, command: Command) -> Result<StatusByte, Error<SPI::Error>> {
        let command_byte = command.to_byte();
        let mut read_buffer = [0; 1];
        self.spi.transfer(&mut read_buffer, &[command_byte | self.addr.cmd_byte()]).map_err(Error::SPIError)?;
        Ok(StatusByte(read_buffer[0]))
    }

    /*fn static_read(&mut self, register: Register) -> Result<StatusByte, Error<SPI::Error>> {
        let mut read_buffer = [0; 1];
        self.spi.transfer(&mut read_buffer, &[Command::StaticRead.to_byte() | register.address() | self.addr.cmd_bytes()]).map_err(Error::SPIError)?;
        Ok(StatusByte(read_buffer[0]))
    }

    fn incremental_read(&mut self, start_register: Register, length: usize) -> Result<StatusByte, Error<SPI::Error>> {
        let mut read_buffer = [0; 1];
        self.spi.transfer(&mut read_buffer, &[Command::IncrementalRead.to_byte() | start_register.address() | self.addr.cmd_bytes()]).map_err(Error::SPIError)?;
        self.spi.transfer(data).map_err(Error::SPIError)?;
        Ok(StatusByte(read_buffer[0]))

    }*/

    fn command_byte<R: config::Register>(&self, cmd: Command) -> u8 {
        cmd.to_byte() | self.addr.cmd_byte() | (R::address() << 2)
    }

    /*fn write_incremental<R: config::Register, const N: usize>(&mut self, data: [u8; N]) -> Result<StatusByte, Error<SPI::Error>> {
        let mut read_buffer = [0u8; 1];
        let mut write_buffer = [0u8; 1 + N];
        write_buffer[0] = Command::IncrementalWrite.to_byte() | self.addr.cmd_byte() | (R::address() << 2);
        write_buffer[1..].copy_from_slice(&data);
        write_buffer[0] = Command::IncrementalWrite.to_byte() | self.addr.cmd_byte() | (config::RegisterConfig0::address() << 2);

        self.spi.transfer(&mut read_buffer, &write_buffer).map_err(Error::SPIError)?;

        self.config = config;

        Ok(StatusByte(read_buffer[0]))
    }*/

    fn transfer<const N: usize>(&mut self, data: [u8; N]) -> Result<StatusByte, Error<SPI::Error>> {
        let mut read_buffer = [0u8; N];
        self.spi.transfer(&mut read_buffer, &data).map_err(Error::SPIError)?;
        Ok(StatusByte(read_buffer[0]))
    }

    /// Reads the ADCDATA register, which returns the latest conversion result.
    /// This function doesn't trigger a conversion, it just reads the register.
    pub fn read_adc_register_data(&mut self) -> Result<i32, Error<SPI::Error>> {
        let mut read_buffer = [0u8; 5];

        // Make sure that the register actually contains the ADC data
        match self.config.irq.irq_mode() {
            IRQMode::MDATOut_IRQ_High | IRQMode::MDATOut_IRQ_Z => {
                return Err(Error::MdatOutSelected);
            },
            _ => {},
        }

        let register_length = match self.config.config3.data_format() {
            config::DataFormat::Size24 => 3,
            _ => 4,
        };
        
        let cmd_byte = [Command::StaticRead.to_byte() | self.addr.cmd_byte() | (0x0 << 2)];
        self.spi.transfer(&mut read_buffer[0..register_length+1], &cmd_byte).map_err(Error::SPIError)?;
        let status_byte = StatusByte(read_buffer[0]);
        if !status_byte.is_data_ready() {
            return Err(Error::DataNotReady);
        }

        Ok(convert_adc_data(&read_buffer[1..register_length+1], self.config.config3.data_format()))
    }

    /// Performs a single-shot conversion with the given MUX setting.
    /// It is recommended to check for DataNotReady errors and retry reading the ADC data
    /// by calling read_adc_register_data() in case the data is not yet ready.
    pub fn single_shot(&mut self, mux: RegisterMux) -> Result<i32, Error<SPI::Error>> {
        if self.config.mux != mux {
            self.write_register(mux)?;
            self.config.mux = mux;
        }

        self.fast_command(Command::Conversion)?;
        self.read_adc_register_data()
    }

    /// Write to a single register of the device.
    /// This might invalidate the current configuration, so it is recommended to call `read_config` afterwards.
    pub fn write_register<R: Register>(&mut self, register: R) -> Result<StatusByte, Error<SPI::Error>> {
        // The maximum size of a register is 4 bytes
        let mut write_buffer = [0u8; 5];

        if R::WRTIABLE {
            return Err(Error::RegisterReadOnly);
        }

        if self.config.lock.is_locked() {
            return Err(Error::DeviceLocked);
        }

        let size = R::size();
        register.to_bytes((&mut write_buffer[1..1+size]).try_into().unwrap());
        write_buffer[0] = self.command_byte::<R>(Command::IncrementalWrite);

        self.transfer(write_buffer)
    }

    pub fn unlock(&mut self) -> Result<(), Error<SPI::Error>> {
        self.config.lock.unlock();
        self.write_register(self.config.lock.clone())?;
        Ok(())
    }

    pub fn lock(&mut self) -> Result<(), Error<SPI::Error>> {
        self.config.lock.lock();
        self.write_register(self.config.lock.clone())?;
        Ok(())
    }

    /// Fully configure the ADC with the given configuration in one write cycle.
    pub fn configure(&mut self, config: config::Config) -> Result<StatusByte, Error<SPI::Error>> {
        let mut write_buffer = [0u8; 24];
        config.to_bytes((&mut write_buffer[1..24]).try_into().unwrap());
        write_buffer[0] = self.command_byte::<config::RegisterConfig0>(Command::IncrementalWrite);

        let result = self.transfer(write_buffer)?;
        self.config = config;

        Ok(result)
    }

    /// Reads the current configuration from the device and stores it in the struct.
    pub fn read_config(&mut self) -> Result<config::Config, Error<SPI::Error>> {
        let mut read_buffer = [0u8; 24];
        let write_buffer = [self.command_byte::<config::RegisterConfig0>(Command::IncrementalWrite)];

        self.spi.transfer(&mut read_buffer, &write_buffer).map_err(Error::SPIError)?;
        self.config = Config::from_bytes((&read_buffer[1..24]).try_into().unwrap()).ok_or(Error::ConfigReadError)?;

        Ok(self.config.clone())
    }

    /// Returns the current configuration of the device, as it is stored in the struct.
    /// As long as the configuration has not been changed, this should be the same as the configuration
    /// stored in the device, but this method does not read the configuration from the device.
    /// If the device is being accessed from multiple controllers, use `read_config` to make sure
    /// that the configuration is up-to-date.
    pub fn config(&self) -> &config::Config {
        &self.config
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    fn parse_binary_string(s: &str) -> Vec<u8> {
        let s = s.replace(" ", "").replace("_", "");
        let length_ceil = (s.len() + 7) / 8;
        let mut result = Vec::with_capacity(length_ceil);
        for i in 0..length_ceil {
            let start = i * 8;
            let end = (i + 1) * 8;
            let slice = &s[start..end];
            result.push(u8::from_str_radix(slice, 2).unwrap());
        }
        result
    }

    fn check_binary_string(s: &str, expected: i32, data_format: config::DataFormat) {
        let data: Vec<u8> = parse_binary_string(s);
        let result = convert_adc_data(&data, data_format);
        assert_eq!(result, expected);
    }

    #[test]
    fn test_data_conversion() {
        let data = parse_binary_string("000000000000000000000001");
        println!("{:?}", data);

        check_binary_string("01111111 11111111 11111111", 8388607, config::DataFormat::Size24);
        check_binary_string("01111111 11111111 11111110", 8388606, config::DataFormat::Size24);
        check_binary_string("00000000 00000000 00000001", 1, config::DataFormat::Size24);
        check_binary_string("00000000 00000000 00000000", 0, config::DataFormat::Size24);
        check_binary_string("11111111 11111111 11111111", -1, config::DataFormat::Size24);
        check_binary_string("10000000 00000000 00000001", -8388607, config::DataFormat::Size24);
        check_binary_string("10000000 00000000 00000000", -8388608, config::DataFormat::Size24);
        
        check_binary_string("01111111 11111111 11111111 00000000", 8388607, config::DataFormat::Size32Data24Left);
        check_binary_string("01111111 11111111 11111110 00000000", 8388606, config::DataFormat::Size32Data24Left);
        check_binary_string("00000000 00000000 00000001 00000000", 1, config::DataFormat::Size32Data24Left);
        check_binary_string("00000000 00000000 00000000 00000000", 0, config::DataFormat::Size32Data24Left);
        check_binary_string("11111111 11111111 11111111 00000000", -1, config::DataFormat::Size32Data24Left);
        check_binary_string("10000000 00000000 00000001 00000000", -8388607, config::DataFormat::Size32Data24Left);
        check_binary_string("10000000 00000000 00000000 00000000", -8388608, config::DataFormat::Size32Data24Left);

        check_binary_string("00000000 11111111 11111111 11111111", 16777215, config::DataFormat::Size32Data25);
        check_binary_string("00000000 01111111 11111111 11111111", 8388607, config::DataFormat::Size32Data25);
        check_binary_string("00000000 00000000 00000000 00000001", 1, config::DataFormat::Size32Data25);
        check_binary_string("00000000 00000000 00000000 00000000", 0, config::DataFormat::Size32Data25);
        check_binary_string("11111111 11111111 11111111 11111111", -1, config::DataFormat::Size32Data25);
        check_binary_string("11111111 10000000 00000000 00000001", -8388607, config::DataFormat::Size32Data25);
        check_binary_string("11111111 01111111 11111111 11111111", -8388609, config::DataFormat::Size32Data25);
        check_binary_string("11111111 00000000 00000000 00000001", -16777215, config::DataFormat::Size32Data25);

        check_binary_string("10100000 11111111 11111111 11111111", 16777215, config::DataFormat::Size32Data25WithId);
        check_binary_string("10100000 01111111 11111111 11111111", 8388607, config::DataFormat::Size32Data25WithId);
        check_binary_string("10100000 00000000 00000000 00000001", 1, config::DataFormat::Size32Data25WithId);
        check_binary_string("10100000 00000000 00000000 00000000", 0, config::DataFormat::Size32Data25WithId);
        check_binary_string("10101111 11111111 11111111 11111111", -1, config::DataFormat::Size32Data25WithId);
        check_binary_string("10101111 10000000 00000000 00000001", -8388607, config::DataFormat::Size32Data25WithId);
        check_binary_string("10101111 01111111 11111111 11111111", -8388609, config::DataFormat::Size32Data25WithId);
        check_binary_string("10101111 00000000 00000000 00000001", -16777215, config::DataFormat::Size32Data25WithId);
    }
}
