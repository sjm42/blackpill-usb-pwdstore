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

pub struct MyUsbSerial {
    pub serial: UsbdSerial,
}

impl fmt::Write for MyUsbSerial {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        s.bytes()
            .try_for_each(|c| {
                if c == b'\n' {
                    self.serial.write(&[b'\r'])?;
                }
                self.serial.write(&[c]).map(|_| ())
            })
            .map_err(|_| fmt::Error)
    }
}

const ROW_SZ: usize = 0x40; // 64
pub fn hex_dump<O>(output: &mut O, addr: usize, buf: &[u8])
where
    O: fmt::Write,
{
    for (i, c) in buf.iter().enumerate() {
        if i % ROW_SZ == 0 {
            if i > 0 {
                write!(output, "\r\n").ok();
            }
            write!(output, "#{:06x} ", addr + i).ok();
        }
        write!(output, "{c:02x} ").ok();
    }
    write!(output, "\r\n").ok();
}

// EOF
