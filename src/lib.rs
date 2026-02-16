// lib.rs
#![no_std]

extern crate alloc;
extern crate no_std_compat as std;
pub use alloc::string::*;
pub use core::fmt::{self, Write};
pub use rand::prelude::*;
pub use rand::SeedableRng;
pub use std::prelude::v1::*;
pub use stm32f4xx_hal as hal;

pub use hal::otg_fs::{UsbBus, UsbBusType, USB};
pub use hal::prelude::*;
pub use hal::watchdog::IndependentWatchdog;
pub use hal::{gpio::*, pac::USART1, serial, spi::*};

pub mod spi_memory;

pub mod pwdstore;
pub use pwdstore::*;

pub mod climenu;
pub use climenu::*;

pub type SerialTx = serial::Tx<USART1, u8>;
pub type UsbdSerial = usbd_serial::SerialPort<'static, UsbBusType>;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum InitState {
    Idle,
    AskPass1,
    AskPass2,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum OpenState {
    Idle,
    AskPass,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum StoreState {
    Idle,
    AskUser,
    AskPass1,
    AskPass2,
}

/// Writer that sends bytes to USB serial via raw pointer.
/// RTIC priority locking guarantees exclusive access.
pub struct CliWriter {
    pub serial: *mut UsbdSerial,
}
unsafe impl Send for CliWriter {}

impl embedded_io::ErrorType for CliWriter {
    type Error = core::convert::Infallible;
}

impl embedded_io::Write for CliWriter {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let serial = unsafe { &mut *self.serial };
        for &b in buf {
            let _ = serial.write(&[b]);
        }
        Ok(buf.len())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// Raw pointer wrapper for USB handler to access serial port
pub struct SerialAccess(pub *mut UsbdSerial);
unsafe impl Send for SerialAccess {}

/// Type alias for the CLI
pub type AppCli = embedded_cli::cli::Cli<
    CliWriter,
    core::convert::Infallible,
    &'static mut [u8],
    &'static mut [u8],
>;

const ROW_SZ: usize = 0x40; // 64
pub fn hex_dump<O>(output: &mut O, addr: usize, buf: &[u8])
where
    O: fmt::Write,
{
    for (i, c) in buf.iter().enumerate() {
        if i % ROW_SZ == 0 {
            if i > 0 {
                writeln!(output).ok();
            }
            write!(output, "#{:06x} ", addr + i).ok();
        }
        write!(output, "{c:02x} ").ok();
    }
    writeln!(output).ok();
}

// EOF
