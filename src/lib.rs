#![cfg_attr(not(feature = "std"), no_std)]
//! Platform-agnostic driver for the Microchip
//! MCP3561, MCP3562, MCP3564, and MCP3561R, MCP3562R, MCP3564R 24-bit Delta-Sigma ADCs.
//! 
//! Implementation is based on the datasheet of the MCP356xR (which includes a voltage
//! reference), but should also work with the older MCP356x chips.


use embedded_hal::spi::{Mode, SpiDevice};

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
}


pub enum Channel {
    Channel0,
    Channel1,
    Channel2,
    Channel3,
    Channel4,
    Channel5,
    Channel6,
    Channel7,
}



/// The hard-coded device address. The factory default is 0b01,
/// but other addresses are available upon request from Microchip.
enum DeviceAddress {
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
    fn cmd_bytes(&self) -> u8 {
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

struct StatusByte(u8);

impl StatusByte {
    fn is_data_ready(&self) -> bool {
        self.0 & 0b0000_0100 == 0
    }

    fn is_crc_error(&self) -> bool {
        self.0 & 0b0000_0010 == 0
    }

    /// The POR bit is set to 0 after a power-on reset, and 1 for each subsequent read.
    fn por(&self) -> bool {
        self.0 & 0b0000_0001 == 0
    }
}




pub struct MCP356x<SPI: SpiDevice> {
    spi: SPI,
    addr: DeviceAddress,
}

impl<SPI: SpiDevice> MCP356x<SPI> {
    pub fn new(spi: SPI) -> Self {
        MCP356x {
            spi,
            addr: DeviceAddress::default(),
        }
    }

    pub fn new_with_address(spi: SPI, addr: DeviceAddress) -> Self {
        MCP356x {
            spi,
            addr,
        }
    }

    fn fast_command(&mut self, command: Command) -> Result<StatusByte, Error<SPI::Error>> {
        let command_byte = command.to_byte();
        let mut read_buffer = [0; 1];
        self.spi.transfer(&mut read_buffer, &[command_byte | self.addr.cmd_bytes()]).map_err(Error::SPIError)?;
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

    fn transfer<const N: usize>(&mut self, data: [u8; N]) -> Result<(), Error<SPI::Error>> {
        let mut read_buffer = [0u8; N];
        self.spi.transfer(&mut read_buffer, &data).map_err(Error::SPIError)
    }

    /// Oneshot reading of the voltage on the given channel
    pub fn read_voltage(&mut self) -> Result<f32, Error<SPI::Error>> {
        let mut buffer = [0u8; 3];
        self.spi.transfer(&mut buffer).map_err(Error::SPIError)?;
        let value = u32::from_be_bytes(buffer);
        let voltage = value as f32 * 2.5 / 0x00FF_FFFF as f32;
        Ok(voltage)
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        
    }
}
