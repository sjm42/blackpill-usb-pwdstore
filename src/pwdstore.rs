// pwdstore.rs
#![allow(dead_code)]

extern crate alloc;
extern crate no_std_compat as std;

use alloc::string::String;
use core::fmt;
use cortex_m::prelude::_embedded_hal_watchdog_Watchdog;
use no_std_compat::borrow::Cow;
use privatebox::PrivateBox;
use rand::{prelude::*, SeedableRng};
use sha2::{Digest, Sha256};
use spi_memory::{series25::Flash, BlockDevice, Read};
use std::prelude::v1::*;
use std::str;

use stm32f4xx_hal as hal;

use hal::watchdog::IndependentWatchdog;
use hal::{gpio::*, pac::SPI1, spi::*};

type MyFlash = Flash<
    Spi<
        SPI1,
        (
            Pin<Alternate<PushPull, 5>, 'A', 5>,
            Pin<Alternate<PushPull, 5>, 'A', 6>,
            Pin<Alternate<PushPull, 5>, 'A', 7>,
        ),
        TransferModeNormal,
    >,
    ErasedPin<Output<PushPull>>,
>;

pub const FLASH_SIZE: usize = 8 * 1024 * 1024; // 8 MB
pub const FLASH_BLOCK_SIZE: usize = 4096;

const ADDR_MASTER_KEY: u32 = 0x000000;
const ADDR_MIN: u32 = FLASH_BLOCK_SIZE as u32;
const ADDR_MAX: u32 = (FLASH_BLOCK_SIZE * 0x07FF) as u32;

const PWD_NAME_LEN: usize = 250;
const PWD_DATA_LEN: usize = FLASH_BLOCK_SIZE - 6 - PWD_NAME_LEN;
const SIZE_PWD_REPR: usize = 6 + PWD_NAME_LEN + PWD_DATA_LEN;

// make it exactly the block size
#[repr(C)]
pub struct PwdRepr {
    len_name: u16,
    len_username: u16,
    len_password: u16,
    name: [u8; PWD_NAME_LEN],
    data: [u8; PWD_DATA_LEN],
}

const SIZE_SCAN_BUF: usize = 2 + PWD_NAME_LEN + 2;
#[repr(C)]
struct ScanBuf {
    len_name: u16,
    len_username: u16,
    len_password: u16,
    name: [u8; PWD_NAME_LEN],
}

impl PwdRepr {
    pub fn new(name: &[u8], username: &[u8], password: &[u8]) -> Option<Self> {
        if name.len() > PWD_NAME_LEN || username.len() + password.len() > PWD_DATA_LEN {
            return None;
        }

        let mut namebuf = [0; PWD_NAME_LEN];
        name.iter().enumerate().for_each(|(i, b)| {
            namebuf[i] = *b;
        });

        let mut databuf = [0; PWD_DATA_LEN];
        username.iter().enumerate().for_each(|(i, b)| {
            databuf[i] = *b;
        });
        let pass_offset = username.len();
        password.iter().enumerate().for_each(|(i, b)| {
            databuf[pass_offset + i] = *b;
        });

        Some(Self {
            len_name: name.len() as u16,
            len_username: username.len() as u16,
            len_password: password.len() as u16,
            name: namebuf,
            data: databuf,
        })
    }

    #[inline(always)]
    pub fn is_valid(&self) -> bool {
        self.len_name > 0
            && self.len_name <= PWD_NAME_LEN as u16
            && self.len_username > 0
            && self.len_password > 0
            && self.len_username + self.len_password <= PWD_DATA_LEN as u16
    }

    #[inline(always)]
    pub fn bytes(&self) -> &[u8] {
        unsafe {
            ::core::slice::from_raw_parts((self as *const PwdRepr) as *const u8, SIZE_PWD_REPR)
        }
    }

    #[inline(always)]
    pub fn bytes_mut(&mut self) -> &mut [u8] {
        unsafe {
            ::core::slice::from_raw_parts_mut((self as *mut PwdRepr) as *mut u8, SIZE_PWD_REPR)
        }
    }

    #[inline(always)]
    pub fn get_name(&self) -> Option<&[u8]> {
        if self.is_valid() {
            Some(&self.name[..self.len_name as usize])
        } else {
            None
        }
    }

    #[inline(always)]
    pub fn get_name_string(&self) -> Option<Cow<'_, str>> {
        if self.is_valid() {
            Some(String::from_utf8_lossy(
                &self.name[..self.len_name as usize],
            ))
        } else {
            None
        }
    }

    #[inline(always)]
    pub fn get_username(&self) -> Option<&[u8]> {
        if self.is_valid() {
            Some(&self.data[..self.len_username as usize])
        } else {
            None
        }
    }

    #[inline(always)]
    pub fn get_password(&self) -> Option<&[u8]> {
        if self.is_valid() {
            let offset = self.len_username as usize;
            Some(&self.data[offset..offset + self.len_password as usize])
        } else {
            None
        }
    }
}

impl ScanBuf {
    #[inline(always)]
    pub fn new() -> Self {
        ScanBuf {
            len_name: 0,
            len_username: 0,
            len_password: 0,
            name: [0; PWD_NAME_LEN],
        }
    }

    #[inline(always)]
    pub fn len_name_is_valid(&self) -> bool {
        self.len_name > 0 && self.len_name <= PWD_NAME_LEN as u16
    }

    #[inline(always)]
    pub fn len_data_is_valid(&self) -> bool {
        self.len_username > 0
            && self.len_password > 0
            && self.len_username + self.len_password <= PWD_DATA_LEN as u16
    }

    #[inline(always)]
    pub fn is_valid(&self) -> bool {
        self.len_name_is_valid() && self.len_data_is_valid()
    }

    #[inline(always)]
    pub fn bytes(&self) -> &[u8] {
        unsafe {
            ::core::slice::from_raw_parts((self as *const ScanBuf) as *const u8, SIZE_SCAN_BUF)
        }
    }

    #[inline(always)]
    pub fn bytes_mut(&mut self) -> &mut [u8] {
        unsafe {
            ::core::slice::from_raw_parts_mut((self as *mut ScanBuf) as *mut u8, SIZE_SCAN_BUF)
        }
    }

    #[inline(always)]
    pub fn len_bytes(&self) -> &[u8] {
        unsafe { ::core::slice::from_raw_parts((self as *const ScanBuf) as *const u8, 6) }
    }

    #[inline(always)]
    pub fn len_bytes_mut(&mut self) -> &mut [u8] {
        unsafe { ::core::slice::from_raw_parts_mut((self as *mut ScanBuf) as *mut u8, 6) }
    }

    #[inline(always)]
    pub fn get_name(&self) -> Option<&[u8]> {
        if self.is_valid() {
            Some(&self.name[..self.len_name as usize])
        } else {
            None
        }
    }

    #[inline(always)]
    pub fn get_name_string(&self) -> Option<Cow<'_, str>> {
        if self.is_valid() {
            Some(String::from_utf8_lossy(
                &self.name[..self.len_name as usize],
            ))
        } else {
            None
        }
    }
}

pub struct PwdStore {
    pub flash: MyFlash,
    pub watchdog: IndependentWatchdog,
    blocks: usize,
    pbox: Option<PrivateBox<StdRng>>,
    rng: Option<StdRng>,
}

impl PwdStore {
    pub fn new(flash: MyFlash, watchdog: IndependentWatchdog) -> Self {
        Self {
            flash,
            watchdog,
            blocks: FLASH_SIZE / FLASH_BLOCK_SIZE,
            pbox: None,
            rng: None,
        }
    }

    pub fn status<D>(&mut self, debug: &mut D)
    where
        D: Sized + fmt::Write,
    {
        write!(
            debug,
            "*** {} ***\r\n\
            Version: {}\r\n\
            Source timestamp: {}\r\n\
            Compiler: {}\r\n\
            Status: {}\r\n",
            env!("CARGO_PKG_NAME"),
            env!("CARGO_PKG_VERSION"),
            env!("SOURCE_TIMESTAMP"),
            env!("RUSTC_VERSION"),
            if self.pbox.is_some() {
                "OPEN"
            } else {
                "CLOSED"
            }
        )
        .ok();
    }

    pub fn init<D>(&mut self, debug: &mut D, master_pass: &str, seed: u64)
    where
        D: Sized + fmt::Write,
    {
        let mut pwd_buf = PwdRepr::new(&[], &[], &[]).unwrap();
        self.flash
            .read(ADDR_MASTER_KEY, pwd_buf.bytes_mut())
            .unwrap();
        if pwd_buf.is_valid() {
            write!(
                debug,
                "Error: will not overwrite existing master key.\r\n\
                If you want a fresh start, issue these commands from main menu:\r\n\
                flash\r\n\
                erase 0 0x800000\r\n\
                exit\r\n"
            )
            .ok();
            return;
        }
        drop(pwd_buf);

        // Create a new RNG seeded with master_pass and given seed (timestamp)
        let mut master_seed = [0u8; privatebox::KEY_SIZE];
        let mut seed_hasher = Sha256::new();
        seed_hasher.update(unsafe {
            ::core::slice::from_raw_parts((&seed as *const u64) as *const u8, 8)
        });
        seed_hasher.update(master_pass.as_bytes());
        master_seed.copy_from_slice(seed_hasher.finalize().as_slice());
        let mut rng_master = StdRng::from_seed(master_seed);

        // Save another rng for later use
        self.rng = Some(StdRng::from_rng(&mut rng_master).unwrap());

        // Create sha2-256 hash from master_pass -- 32 bytes
        let mut pass_hash = [0; privatebox::KEY_SIZE];
        let mut pass_hasher = Sha256::new();
        pass_hasher.update(master_pass.as_bytes());
        pass_hash.copy_from_slice(pass_hasher.finalize().as_slice());

        // Generate new master key, 32 bytes, 256 bits
        let mut master_key = [0; privatebox::KEY_SIZE];
        rng_master.fill_bytes(&mut master_key);

        // Make a new PrivateBox with the key, i.e. open the password store
        let rng_runtime = StdRng::from_rng(&mut rng_master).unwrap();
        let pb_runtime = PrivateBox::new(&master_key, rng_runtime);
        self.pbox = Some(pb_runtime);

        // Encrypt the master key using master_pass as encryption key
        let mut pb_master = PrivateBox::new(&pass_hash, rng_master);
        let master_enc = pb_master.encrypt(&master_key, &[], &[]).unwrap();
        drop(pb_master);

        // Store the encrypted master key onto flash
        let mut flash_pwd =
            PwdRepr::new("MASTER".as_bytes(), "MASTER".as_bytes(), &master_enc).unwrap();
        self.flash.erase_sectors(ADDR_MASTER_KEY, 1).unwrap();
        self.flash
            .write_bytes(ADDR_MASTER_KEY, flash_pwd.bytes_mut())
            .unwrap();
        write!(
            debug,
            "New master key generated & saved successfully. Password store opened.\r\n"
        )
        .ok();
    }

    pub fn open<D>(&mut self, debug: &mut D, master_pass: &str, seed: u64)
    where
        D: Sized + fmt::Write,
    {
        let mut pass_hash = [0; privatebox::KEY_SIZE];
        let mut pass_hasher = Sha256::new();
        pass_hasher.update(master_pass.as_bytes());
        pass_hash.copy_from_slice(pass_hasher.finalize().as_slice());
        let rng_master = StdRng::seed_from_u64(0);
        let pb_master = PrivateBox::new(&pass_hash, rng_master);

        let mut pwd_buf = PwdRepr::new(&[], &[], &[]).unwrap();
        self.flash
            .read(ADDR_MASTER_KEY, pwd_buf.bytes_mut())
            .unwrap();
        let pwd_crypted = match pwd_buf.get_password() {
            Some(d) => d,
            None => {
                write!(debug, "No master key saved on flash.\r\n").ok();
                return;
            }
        };

        let master_vec = match pb_master.decrypt(pwd_crypted, &[]) {
            Ok((decr, _auth_h)) => decr,
            Err(e) => {
                write!(debug, "Decrypt failed: {e:?}\r\n").ok();
                return;
            }
        };
        drop(pb_master);
        let mut master_key = [0; privatebox::KEY_SIZE];
        master_key.copy_from_slice(&master_vec);

        // Create a new RNG seeded with master_pass and given seed
        let mut runtime_seed = [0u8; privatebox::KEY_SIZE];
        let mut seed_hasher = Sha256::new();
        seed_hasher.update(unsafe {
            ::core::slice::from_raw_parts((&seed as *const u64) as *const u8, 8)
        });
        seed_hasher.update(&master_key);
        runtime_seed.copy_from_slice(seed_hasher.finalize().as_slice());
        let mut rng_runtime = StdRng::from_seed(runtime_seed);

        // Save another rng for later use
        self.rng = Some(StdRng::from_rng(&mut rng_runtime).unwrap());

        let pb_runtime = PrivateBox::new(&master_key, rng_runtime);
        self.pbox = Some(pb_runtime);
        write!(
            debug,
            "Master key decrypted successfully. Password store opened.\r\n"
        )
        .ok();
    }

    pub fn close<D>(&mut self, debug: &mut D)
    where
        D: Sized + fmt::Write,
    {
        self.pbox = None;
        self.rng = None;
        write!(debug, "Password store closed.\r\n").ok();
    }

    pub fn store<D>(&mut self, debug: &mut D, name: &str, username: &str, password: &str)
    where
        D: Sized + fmt::Write,
    {
        let pbox = match &mut self.pbox {
            None => {
                write!(debug, "Error: password store is not open.\r\n").ok();
                return;
            }
            Some(pb) => pb,
        };

        // encrypt
        let encr_user = pbox.encrypt(username.as_bytes(), &[], &[]).unwrap();
        let encr_pass = pbox.encrypt(password.as_bytes(), &[], &[]).unwrap();

        write!(
            debug,
            "Encrypted into {}+{} bytes\r\n",
            encr_user.len(),
            encr_pass.len()
        )
        .ok();

        let addr = match self.find_free(debug) {
            Some(a) => a,
            None => {
                write!(debug, "Error: cannot find free location!\r\n").ok();
                return;
            }
        };

        let mut pwd = match PwdRepr::new(name.as_bytes(), &encr_user, &encr_pass) {
            Some(p) => p,
            None => {
                write!(debug, "Encryption error: username+password too long.\r\n").ok();
                return;
            }
        };
        self.flash.erase_sectors(addr as u32, 1).unwrap();
        self.flash
            .write_bytes(addr as u32, pwd.bytes_mut())
            .unwrap();
        write!(debug, "Stored to loc 0x{:03x}\r\n", addr / FLASH_BLOCK_SIZE).ok();
    }

    pub fn fetch<D>(&mut self, debug: &mut D, loc: usize)
    where
        D: Sized + fmt::Write,
    {
        let pbox = match &mut self.pbox {
            None => {
                write!(debug, "Error: password store is not open.\r\n").ok();
                return;
            }
            Some(pb) => pb,
        };

        let addr = (FLASH_BLOCK_SIZE * loc) as u32;
        if !(ADDR_MIN..=ADDR_MAX).contains(&addr) {
            write!(debug, "Error: illegal location.\r\n").ok();
            return;
        }

        let mut pwd_buf = PwdRepr::new(&[], &[], &[]).unwrap();
        self.flash.read(addr, pwd_buf.bytes_mut()).unwrap();
        if !pwd_buf.is_valid() {
            write!(debug, "No valid data in this location.\r\n").ok();
            return;
        }
        let encr_user = pwd_buf.get_username().unwrap();
        let encr_pass = pwd_buf.get_password().unwrap();

        // decrypt it
        let metadata = [];
        let decr_user = match pbox.decrypt(encr_user, &metadata) {
            Err(e) => {
                write!(debug, "Username decrypt failed: {e:?}\r\n").ok();
                return;
            }
            Ok((dec, _auth_h)) => dec,
        };
        let decr_pass = match pbox.decrypt(encr_pass, &metadata) {
            Err(e) => {
                write!(debug, "Password decrypt failed: {e:?}\r\n").ok();
                return;
            }
            Ok((dec, _auth_h)) => dec,
        };

        let name = pwd_buf.get_name_string().unwrap();
        let str_user = String::from_utf8_lossy(&decr_user);
        let str_pass = String::from_utf8_lossy(&decr_pass);
        write!(
            debug,
            "name: {name}\r\n\
            user: {str_user}\r\n\
            pass: {str_pass}\r\n"
        )
        .ok();
    }

    pub fn drop<D>(&mut self, debug: &mut D, loc: usize)
    where
        D: Sized + fmt::Write,
    {
        if self.pbox.is_none() {
            write!(debug, "Error: password store is not open.\r\n").ok();
            return;
        }

        let addr = (FLASH_BLOCK_SIZE * loc) as u32;
        if !(ADDR_MIN..=ADDR_MAX).contains(&addr) {
            write!(debug, "Error: illegal location.\r\n").ok();
            return;
        }

        let mut pwd_buf = PwdRepr::new(&[], &[], &[]).unwrap();
        self.flash.read(addr, pwd_buf.bytes_mut()).unwrap();
        if !pwd_buf.is_valid() {
            write!(debug, "No valid data in this location.\r\n").ok();
            return;
        }

        self.flash.erase_sectors(addr as u32, 1).unwrap();
        write!(debug, "Dropped loc 0x{loc:03x}\r\n").ok();
    }

    pub fn scan<D>(&mut self, debug: &mut D)
    where
        D: Sized + fmt::Write,
    {
        if self.pbox.is_none() {
            write!(debug, "Error: password store is not open.\r\n").ok();
            return;
        }

        let mut scan_buf = ScanBuf::new();

        for i in 0..self.blocks {
            let addr = i * FLASH_BLOCK_SIZE;
            if i % 16 == 0 {
                self.watchdog.feed();
                write!(debug, ".").ok();
            }

            if let Err(e) = self.flash.read(addr as u32, scan_buf.bytes_mut()) {
                write!(
                    debug,
                    "\r\n### Flash read error at 0x{:06x}: {:?}\r\n",
                    addr, e
                )
                .ok();
                continue;
            }

            if scan_buf.is_valid() {
                let name = scan_buf.get_name_string().unwrap();
                write!(
                    debug,
                    "\r\n*** Password found at 0x{addr:06x}, encr {}+{} bytes, loc: 0x{i:03x} name: {}\r\n",
                    scan_buf.len_username,
                    scan_buf.len_password,
                    name.as_ref()
                )
                .ok();
            }
        }
    }

    pub fn find_free_from<D>(&mut self, _debug: &mut D, from_addr: usize) -> Option<usize>
    where
        D: Sized + fmt::Write,
    {
        let mut buf = ScanBuf::new();

        let mut addr = from_addr;
        while addr < FLASH_SIZE {
            self.watchdog.feed();
            self.flash.read(addr as u32, buf.len_bytes_mut()).unwrap();

            // first "invalid" block is free for (re)use
            if !buf.is_valid() {
                return Some(addr);
            }
            addr += FLASH_BLOCK_SIZE;
        }
        None
    }

    pub fn find_free<D>(&mut self, debug: &mut D) -> Option<usize>
    where
        D: Sized + fmt::Write,
    {
        if self.rng.is_none() {
            write!(debug, "Error: no RNG. Open password store first.\r\n").ok();
            return None;
        }

        let rand_loc =
            self.rng.as_mut().unwrap().gen::<usize>() & (FLASH_SIZE / FLASH_BLOCK_SIZE - 1);
        write!(debug, "First rand loc: 0x{:03x}\r\n", rand_loc).ok();
        if let Some(addr) = self.find_free_from(debug, rand_loc * FLASH_BLOCK_SIZE) {
            return Some(addr);
        }
        // did not find free block with random, scan from start of flash...
        self.find_free_from(debug, FLASH_BLOCK_SIZE)
    }

    pub fn list<D>(&mut self, debug: &mut D)
    where
        D: Sized + fmt::Write,
    {
        if self.pbox.is_none() {
            write!(debug, "Error: password store is not open.\r\n").ok();
            return;
        }

        write!(debug, "loc   name\r\n----------\r\n").ok();
        for (addr, name) in self {
            write!(debug, "0x{:03x} {}\r\n", addr / FLASH_BLOCK_SIZE, &name).ok();
        }
    }

    pub fn search<D>(&mut self, debug: &mut D, srch: &[u8])
    where
        D: Sized + fmt::Write,
    {
        if self.pbox.is_none() {
            write!(debug, "Error: password store is not open.\r\n").ok();
            return;
        }

        let len = match srch.len() {
            0 => {
                return;
            }
            len => len,
        };

        write!(debug, "loc   name\r\n----------\r\n").ok();
        for (addr, name) in self {
            if name.as_bytes().windows(len).any(|window| window == srch) {
                write!(debug, "0x{:03x} {}\r\n", addr / FLASH_BLOCK_SIZE, &name).ok();
            }
        }
    }
}

pub struct PwdStoreIterator<'a> {
    pwd_store: &'a mut PwdStore,
    addr: usize,
    buf: ScanBuf,
}

impl<'a> IntoIterator for &'a mut PwdStore {
    type Item = (usize, String);
    type IntoIter = PwdStoreIterator<'a>;
    fn into_iter(self) -> Self::IntoIter {
        PwdStoreIterator {
            pwd_store: self,
            addr: 0,
            buf: ScanBuf::new(),
        }
    }
}

impl<'a> Iterator for PwdStoreIterator<'a> {
    type Item = (usize, String);
    fn next(&mut self) -> Option<(usize, String)> {
        while self.addr < FLASH_SIZE {
            // just read the len fields for validation
            self.pwd_store
                .flash
                .read(self.addr as u32, self.buf.len_bytes_mut())
                .unwrap();
            if !self.buf.is_valid() {
                // short circuit now, nothing more to see at this address
                self.addr += FLASH_BLOCK_SIZE;
                continue;
            }

            self.pwd_store.watchdog.feed();

            // Read the whole ScanBuf from flash
            self.pwd_store
                .flash
                .read(self.addr as u32, self.buf.bytes_mut())
                .unwrap();

            let ret_addr = self.addr;
            self.addr += FLASH_BLOCK_SIZE;

            if self.buf.is_valid() {
                let name = self.buf.get_name_string().unwrap().to_string();
                return Some((ret_addr, name));
            }
        }
        None
    }
}
// EOF
