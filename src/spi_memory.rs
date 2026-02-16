// spi_memory.rs
//
// Vendored and updated from https://github.com/sjm42/spi-memory
// Originally based on the spi-memory crate (now archived).
// Updated for embedded-hal 1.0 traits and made generic over delay type.

#![allow(dead_code)]

use core::convert::TryInto;
use core::fmt::{self, Debug, Display};

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiBus;

// ---- Error type ----

pub enum Error<SPI: SpiBus<u8>, GPIO: OutputPin> {
    Spi(SPI::Error),
    Gpio(GPIO::Error),
    UnexpectedStatus,
}

impl<SPI: SpiBus<u8>, GPIO: OutputPin> Debug for Error<SPI, GPIO>
where
    SPI::Error: Debug,
    GPIO::Error: Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Spi(spi) => write!(f, "Error::Spi({:?})", spi),
            Error::Gpio(gpio) => write!(f, "Error::Gpio({:?})", gpio),
            Error::UnexpectedStatus => f.write_str("Error::UnexpectedStatus"),
        }
    }
}

impl<SPI: SpiBus<u8>, GPIO: OutputPin> Display for Error<SPI, GPIO>
where
    SPI::Error: Display,
    GPIO::Error: Display,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Spi(spi) => write!(f, "SPI error: {}", spi),
            Error::Gpio(gpio) => write!(f, "GPIO error: {}", gpio),
            Error::UnexpectedStatus => f.write_str("unexpected value in status register"),
        }
    }
}

// ---- Traits ----

pub trait Read<Addr, SPI: SpiBus<u8>, CS: OutputPin> {
    fn read(&mut self, addr: Addr, buf: &mut [u8]) -> Result<(), Error<SPI, CS>>;
}

pub trait BlockDevice<Addr, SPI: SpiBus<u8>, CS: OutputPin> {
    fn erase_sectors(&mut self, addr: Addr, amount: usize) -> Result<u32, Error<SPI, CS>>;
    fn erase_all(&mut self) -> Result<(), Error<SPI, CS>>;
    fn write_bytes(&mut self, addr: Addr, data: &mut [u8]) -> Result<u32, Error<SPI, CS>>;
}

// ---- Prelude ----

pub mod prelude {
    pub use super::{BlockDevice, Read};
}

// ---- Identification ----

struct HexSlice<'a>([u8; 3], core::marker::PhantomData<&'a ()>);

impl fmt::Debug for HexSlice<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[{:02x}, {:02x}, {:02x}]", self.0[0], self.0[1], self.0[2])
    }
}

pub struct Identification {
    bytes: [u8; 3],
    continuations: u8,
}

impl Identification {
    pub fn from_jedec_id(buf: &[u8]) -> Identification {
        let mut start_idx = 0;
        for (i, &b) in buf.iter().enumerate().take(buf.len() - 2) {
            if b != 0x7F {
                start_idx = i;
                break;
            }
        }

        Self {
            bytes: [buf[start_idx], buf[start_idx + 1], buf[start_idx + 2]],
            continuations: start_idx as u8,
        }
    }

    pub fn mfr_code(&self) -> u8 {
        self.bytes[0]
    }

    pub fn device_id(&self) -> &[u8] {
        self.bytes[1..].as_ref()
    }

    pub fn continuation_count(&self) -> u8 {
        self.continuations
    }
}

impl fmt::Debug for Identification {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("Identification")
            .field(&HexSlice(self.bytes, core::marker::PhantomData))
            .finish()
    }
}

// ---- Opcodes and Status ----

#[allow(unused)]
enum Opcode {
    ReadDeviceId = 0xAB,
    ReadMfDId = 0x90,
    ReadJedecId = 0x9F,
    WriteEnable = 0x06,
    WriteDisable = 0x04,
    ReadStatus = 0x05,
    WriteStatus = 0x01,
    Read = 0x03,
    PageProg = 0x02,
    SectorErase = 0x20,
    BlockErase = 0xD8,
    ChipErase = 0xC7,
    EnableReset = 0x66,
    ResetDevice = 0x99,
    FastRead = 0x0B,
}

bitflags::bitflags! {
    pub struct Status: u8 {
        const BUSY = 0b00000001;
        const WEL  = 0b00000010;
        const PROT = 0b00011100;
        const SRWD = 0b10000000;
    }
}

// ---- Flash driver ----

pub struct Flash<SPI: SpiBus<u8>, CS: OutputPin, D: DelayNs> {
    spi: SPI,
    cs: CS,
    delay: D,
}

impl<SPI, CS, D> Flash<SPI, CS, D>
where
    SPI: SpiBus<u8>,
    CS: OutputPin,
    D: DelayNs,
{
    pub fn init(spi: SPI, cs: CS, delay: D) -> Result<Self, Error<SPI, CS>> {
        let mut this = Self { spi, cs, delay };
        this.reset_device()?;
        let status = this.read_status()?;

        if !(status & (Status::BUSY | Status::WEL)).is_empty() {
            return Err(Error::UnexpectedStatus);
        }
        Ok(this)
    }

    fn command(&mut self, bytes: &mut [u8]) -> Result<(), Error<SPI, CS>> {
        self.cs.set_low().map_err(Error::Gpio)?;
        let spi_result = self.spi.transfer_in_place(bytes).map_err(Error::Spi);
        self.cs.set_high().map_err(Error::Gpio)?;
        spi_result?;
        Ok(())
    }

    pub fn read_jedec_id(&mut self) -> Result<Identification, Error<SPI, CS>> {
        let mut buf: [u8; 12] = [0; 12];
        buf[0] = Opcode::ReadJedecId as u8;
        self.command(&mut buf)?;
        Ok(Identification::from_jedec_id(&buf[1..]))
    }

    pub fn read_status(&mut self) -> Result<Status, Error<SPI, CS>> {
        let mut buf = [Opcode::ReadStatus as u8, 0];
        self.command(&mut buf)?;
        Ok(Status::from_bits_truncate(buf[1]))
    }

    pub fn read_status_u8(&mut self) -> Result<u8, Error<SPI, CS>> {
        let mut buf = [Opcode::ReadStatus as u8, 0];
        self.command(&mut buf)?;
        Ok(buf[1])
    }

    pub fn write_enable(&mut self) -> Result<(), Error<SPI, CS>> {
        let mut cmd_buf = [Opcode::WriteEnable as u8];
        self.command(&mut cmd_buf)?;
        Ok(())
    }

    pub fn write_disable(&mut self) -> Result<(), Error<SPI, CS>> {
        let mut cmd_buf = [Opcode::WriteDisable as u8];
        self.command(&mut cmd_buf)?;
        Ok(())
    }

    pub fn wait_done(&mut self) -> Result<u32, Error<SPI, CS>> {
        let mut i = 0;
        while self.read_status()?.contains(Status::BUSY) {
            self.delay.delay_us(50);
            i += 1;
        }
        Ok(i)
    }

    pub fn reset_device(&mut self) -> Result<(), Error<SPI, CS>> {
        let mut cmd_buf = [Opcode::EnableReset as u8];
        self.command(&mut cmd_buf)?;
        let mut cmd_buf = [Opcode::ResetDevice as u8];
        self.command(&mut cmd_buf)?;
        self.delay.delay_us(50);
        Ok(())
    }

    pub fn wait_ms(&mut self, ms: u32) {
        self.delay.delay_ms(ms);
    }

    pub fn wait_us(&mut self, us: u32) {
        self.delay.delay_us(us);
    }
}

impl<SPI: SpiBus<u8>, CS: OutputPin, D: DelayNs> Read<u32, SPI, CS> for Flash<SPI, CS, D> {
    fn read(&mut self, addr: u32, buf: &mut [u8]) -> Result<(), Error<SPI, CS>> {
        let mut cmd_buf = [
            Opcode::Read as u8,
            (addr >> 16) as u8,
            (addr >> 8) as u8,
            addr as u8,
        ];

        self.cs.set_low().map_err(Error::Gpio)?;
        let mut spi_result = self.spi.transfer_in_place(&mut cmd_buf).map_err(Error::Spi);
        if spi_result.is_ok() {
            spi_result = self.spi.transfer_in_place(buf).map_err(Error::Spi);
        }
        self.cs.set_high().map_err(Error::Gpio)?;
        spi_result
    }
}

impl<SPI: SpiBus<u8>, CS: OutputPin, D: DelayNs> BlockDevice<u32, SPI, CS>
    for Flash<SPI, CS, D>
{
    fn erase_sectors(&mut self, addr: u32, amount: usize) -> Result<u32, Error<SPI, CS>> {
        for c in 0..amount {
            self.write_enable()?;
            let current_addr: u32 = (addr as usize + c * 256).try_into().unwrap();
            let mut cmd_buf = [
                Opcode::SectorErase as u8,
                (current_addr >> 16) as u8,
                (current_addr >> 8) as u8,
                current_addr as u8,
            ];
            self.command(&mut cmd_buf)?;
            self.delay.delay_ms(40);
        }
        self.wait_done()
    }

    fn write_bytes(&mut self, addr: u32, data: &mut [u8]) -> Result<u32, Error<SPI, CS>> {
        let mut wait = 0u32;
        for (c, chunk) in data.chunks_mut(256).enumerate() {
            self.write_enable()?;
            let current_addr: u32 = (addr as usize + c * 256).try_into().unwrap();
            let mut cmd_buf = [
                Opcode::PageProg as u8,
                (current_addr >> 16) as u8,
                (current_addr >> 8) as u8,
                current_addr as u8,
            ];

            self.cs.set_low().map_err(Error::Gpio)?;
            let mut spi_result = self
                .spi
                .transfer_in_place(&mut cmd_buf)
                .map_err(Error::Spi);
            if spi_result.is_ok() {
                spi_result = self.spi.transfer_in_place(chunk).map_err(Error::Spi);
            }
            self.cs.set_high().map_err(Error::Gpio)?;
            spi_result?;
            self.delay.delay_us(200);
            wait += self.wait_done()?;
        }
        Ok(wait)
    }

    fn erase_all(&mut self) -> Result<(), Error<SPI, CS>> {
        self.write_enable()?;
        let mut cmd_buf = [Opcode::ChipErase as u8];
        self.command(&mut cmd_buf)?;
        self.delay.delay_ms(20_000);
        self.wait_done()?;
        Ok(())
    }
}

// EOF
