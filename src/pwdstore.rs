// pwdstore.rs
#![allow(dead_code)]

extern crate alloc;
extern crate no_std_compat as std;

use alloc::string::*;
use alloc::vec::Vec;
use chacha20poly1305::{
    aead::{Aead, KeyInit},
    XChaCha20Poly1305, XNonce,
};
use core::fmt;
use no_std_compat::borrow::Cow;
use rand::{prelude::*, SeedableRng};
use sha2::{Digest, Sha256};
use crate::spi_memory::{BlockDevice, Flash, Read};
use std::prelude::v1::*;
use std::str;

use stm32f4xx_hal as hal;

use hal::gpio::*;
use hal::pac::{SPI1, TIM5};
use hal::spi::*;
use hal::timer;
use hal::watchdog::IndependentWatchdog;

type MyFlash = Flash<Spi<SPI1>, ErasedPin<Output<PushPull>>, timer::Delay<TIM5, 1_000_000>>;

pub const FLASH_SIZE: usize = 8 * 1024 * 1024; // 8 MB
pub const FLASH_BLOCK_SIZE: usize = 4096;

const ADDR_MASTER_KEY: u32 = 0x000000;
const ADDR_MIN: u32 = FLASH_BLOCK_SIZE as u32;
const ADDR_MAX: u32 = (FLASH_BLOCK_SIZE * 0x07FF) as u32;

const KEY_SIZE: usize = 32;
const NONCE_SIZE: usize = 24;

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

pub struct PwdStore {
    pub flash: MyFlash,
    pub watchdog: IndependentWatchdog,
    cipher: Option<XChaCha20Poly1305>,
    rng: Option<StdRng>,
}

fn encrypt_blob(cipher: &XChaCha20Poly1305, rng: &mut StdRng, plaintext: &[u8]) -> Vec<u8> {
    let mut nonce_bytes = [0u8; NONCE_SIZE];
    rng.fill_bytes(&mut nonce_bytes);
    let nonce = XNonce::from_slice(&nonce_bytes);
    let ct = cipher.encrypt(nonce, plaintext).unwrap();
    let mut blob = Vec::with_capacity(NONCE_SIZE + ct.len());
    blob.extend_from_slice(&nonce_bytes);
    blob.extend_from_slice(&ct);
    blob
}

fn decrypt_blob(
    cipher: &XChaCha20Poly1305,
    ciphertext: &[u8],
) -> Result<Vec<u8>, chacha20poly1305::aead::Error> {
    if ciphertext.len() < NONCE_SIZE {
        return Err(chacha20poly1305::aead::Error);
    }
    let nonce = XNonce::from_slice(&ciphertext[..NONCE_SIZE]);
    cipher.decrypt(nonce, &ciphertext[NONCE_SIZE..])
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

impl PwdStore {
    pub fn new(flash: MyFlash, watchdog: IndependentWatchdog) -> Self {
        Self {
            flash,
            watchdog,
            cipher: None,
            rng: None,
        }
    }

    pub fn status<D>(&mut self, debug: &mut D)
    where
        D: Sized + fmt::Write,
    {
        write!(
            debug,
            "*** {} ***\n\
            Version: {}\n\
            Source timestamp: {}\n\
            Compiler: {}\n\
            Status: {}\n",
            env!("CARGO_PKG_NAME"),
            env!("CARGO_PKG_VERSION"),
            env!("SOURCE_TIMESTAMP"),
            env!("RUSTC_VERSION"),
            if self.cipher.is_some() {
                "OPEN"
            } else {
                "CLOSED"
            }
        )
        .ok();
    }

    pub fn is_initialized(&mut self) -> bool {
        let mut pwd_buf = PwdRepr::new(&[], &[], &[]).unwrap();
        self.flash
            .read(ADDR_MASTER_KEY, pwd_buf.bytes_mut())
            .unwrap();
        pwd_buf.is_valid()
    }

    pub fn init<D>(&mut self, debug: &mut D, master_pass: &str, seed: u64)
    where
        D: Sized + fmt::Write,
    {
        // Create a new RNG seeded with master_pass and given seed (timestamp)
        let mut master_seed = [0u8; KEY_SIZE];
        let mut seed_hasher = Sha256::new();
        seed_hasher.update(unsafe {
            ::core::slice::from_raw_parts((&seed as *const u64) as *const u8, 8)
        });
        seed_hasher.update(master_pass.as_bytes());
        master_seed.copy_from_slice(seed_hasher.finalize().as_slice());
        let mut rng_master = StdRng::from_seed(master_seed);

        // Save another rng for later use
        self.rng = Some(StdRng::from_rng(&mut rng_master));

        // Create sha2-256 hash from master_pass -- 32 bytes
        let mut pass_hash = [0; KEY_SIZE];
        let mut pass_hasher = Sha256::new();
        pass_hasher.update(master_pass.as_bytes());
        pass_hash.copy_from_slice(pass_hasher.finalize().as_slice());

        // Generate new master key, 32 bytes, 256 bits
        let mut master_key = [0; KEY_SIZE];
        rng_master.fill_bytes(&mut master_key);

        // Open the password store with the new master key
        self.cipher = Some(
            XChaCha20Poly1305::new_from_slice(&master_key).unwrap(),
        );

        // Encrypt the master key using master_pass as encryption key
        let cipher_master = XChaCha20Poly1305::new_from_slice(&pass_hash).unwrap();
        let master_enc = encrypt_blob(&cipher_master, &mut rng_master, &master_key);

        // Store the encrypted master key onto flash
        let mut flash_pwd =
            PwdRepr::new("MASTER".as_bytes(), "MASTER".as_bytes(), &master_enc).unwrap();
        self.flash.erase_sectors(ADDR_MASTER_KEY, 1).unwrap();
        self.flash
            .write_bytes(ADDR_MASTER_KEY, flash_pwd.bytes_mut())
            .unwrap();
        writeln!(
            debug,
            "New master key generated & saved successfully. Password store opened."
        )
        .ok();
    }

    pub fn open<D>(&mut self, debug: &mut D, master_pass: &str, seed: u64)
    where
        D: Sized + fmt::Write,
    {
        let mut pass_hash = [0; KEY_SIZE];
        let mut pass_hasher = Sha256::new();
        pass_hasher.update(master_pass.as_bytes());
        pass_hash.copy_from_slice(pass_hasher.finalize().as_slice());
        let cipher_master = XChaCha20Poly1305::new_from_slice(&pass_hash).unwrap();

        let mut pwd_buf = PwdRepr::new(&[], &[], &[]).unwrap();
        self.flash
            .read(ADDR_MASTER_KEY, pwd_buf.bytes_mut())
            .unwrap();
        let pwd_crypted = match pwd_buf.get_password() {
            Some(d) => d,
            None => {
                writeln!(debug, "No master key saved on flash.").ok();
                return;
            }
        };

        let master_vec = match decrypt_blob(&cipher_master, pwd_crypted) {
            Ok(decr) => decr,
            Err(e) => {
                writeln!(debug, "Decrypt failed: {e:?}").ok();
                return;
            }
        };
        let mut master_key = [0; KEY_SIZE];
        master_key.copy_from_slice(&master_vec);

        // Create a new RNG seeded with master_key and given seed
        let mut runtime_seed = [0u8; KEY_SIZE];
        let mut seed_hasher = Sha256::new();
        seed_hasher.update(unsafe {
            ::core::slice::from_raw_parts((&seed as *const u64) as *const u8, 8)
        });
        seed_hasher.update(master_key);
        runtime_seed.copy_from_slice(seed_hasher.finalize().as_slice());
        let mut rng_runtime = StdRng::from_seed(runtime_seed);

        // Save another rng for later use
        self.rng = Some(StdRng::from_rng(&mut rng_runtime));

        self.cipher = Some(
            XChaCha20Poly1305::new_from_slice(&master_key).unwrap(),
        );
        writeln!(
            debug,
            "Master key decrypted successfully. Password store opened."
        )
        .ok();
    }

    pub fn close<D>(&mut self, debug: &mut D)
    where
        D: Sized + fmt::Write,
    {
        self.cipher = None;
        self.rng = None;
        writeln!(debug, "Password store closed.").ok();
    }

    pub fn store<D>(&mut self, debug: &mut D, name: &str, username: &str, password: &str)
    where
        D: Sized + fmt::Write,
    {
        let cipher = match &self.cipher {
            None => {
                writeln!(debug, "Error: password store is not open.").ok();
                return;
            }
            Some(c) => c,
        };
        let rng = self.rng.as_mut().unwrap();

        // encrypt
        let encr_user = encrypt_blob(cipher, rng, username.as_bytes());
        let encr_pass = encrypt_blob(cipher, rng, password.as_bytes());

        let addr = match self.find_free(debug) {
            Some(a) => a,
            None => {
                writeln!(debug, "Error: cannot find free location!").ok();
                return;
            }
        };

        let mut pwd = match PwdRepr::new(name.as_bytes(), &encr_user, &encr_pass) {
            Some(p) => p,
            None => {
                writeln!(debug, "Encryption error: username+password too long.").ok();
                return;
            }
        };

        self.flash.erase_sectors(addr as u32, 1).unwrap();
        self.flash
            .write_bytes(addr as u32, pwd.bytes_mut())
            .unwrap();
        writeln!(debug, "Stored to loc 0x{:03x}", addr / FLASH_BLOCK_SIZE).ok();
    }

    pub fn fetch<D>(&mut self, debug: &mut D, loc: usize)
    where
        D: Sized + fmt::Write,
    {
        let cipher = match &self.cipher {
            None => {
                writeln!(debug, "Error: password store is not open.").ok();
                return;
            }
            Some(c) => c,
        };

        let addr = (FLASH_BLOCK_SIZE * loc) as u32;
        if !(ADDR_MIN..=ADDR_MAX).contains(&addr) {
            writeln!(debug, "Error: illegal location.").ok();
            return;
        }

        let mut pwd_buf = PwdRepr::new(&[], &[], &[]).unwrap();
        self.flash.read(addr, pwd_buf.bytes_mut()).unwrap();
        if !pwd_buf.is_valid() {
            writeln!(debug, "No valid data in this location.").ok();
            return;
        }
        let encr_user = pwd_buf.get_username().unwrap();
        let encr_pass = pwd_buf.get_password().unwrap();

        // decrypt it
        let decr_user = match decrypt_blob(cipher, encr_user) {
            Err(e) => {
                writeln!(debug, "Username decrypt failed: {e:?}").ok();
                return;
            }
            Ok(dec) => dec,
        };
        let decr_pass = match decrypt_blob(cipher, encr_pass) {
            Err(e) => {
                writeln!(debug, "Password decrypt failed: {e:?}").ok();
                return;
            }
            Ok(dec) => dec,
        };

        let name = pwd_buf.get_name_string().unwrap();
        let str_user = String::from_utf8_lossy(&decr_user);
        let str_pass = String::from_utf8_lossy(&decr_pass);
        write!(
            debug,
            "name: {name}\n\
            user: {str_user}\n\
            pass: {str_pass}\n"
        )
        .ok();
    }

    pub fn drop<D>(&mut self, debug: &mut D, loc: usize)
    where
        D: Sized + fmt::Write,
    {
        if self.cipher.is_none() {
            writeln!(debug, "Error: password store is not open.").ok();
            return;
        }

        let addr = (FLASH_BLOCK_SIZE * loc) as u32;
        if !(ADDR_MIN..=ADDR_MAX).contains(&addr) {
            writeln!(debug, "Error: illegal location.").ok();
            return;
        }

        let mut pwd_buf = PwdRepr::new(&[], &[], &[]).unwrap();
        self.flash.read(addr, pwd_buf.bytes_mut()).unwrap();
        if !pwd_buf.is_valid() {
            writeln!(debug, "No valid data in this location.").ok();
            return;
        }

        self.flash.erase_sectors(addr, 1).unwrap();
        writeln!(debug, "Dropped loc 0x{loc:03x}").ok();
    }

    pub fn scan<D>(&mut self, debug: &mut D)
    where
        D: Sized + fmt::Write,
    {
        if self.cipher.is_none() {
            writeln!(debug, "Error: password store is not open.").ok();
            return;
        }

        let mut scan_buf = ScanBuf::new();

        for i in 0..FLASH_SIZE / FLASH_BLOCK_SIZE {
            let addr = i * FLASH_BLOCK_SIZE;
            if i % 16 == 0 {
                self.watchdog.feed();
                write!(debug, ".").ok();
            }

            if let Err(e) = self.flash.read(addr as u32, scan_buf.bytes_mut()) {
                write!(debug, "\n### Flash read error at 0x{addr:06x}: {e:?}\n",).ok();
                continue;
            }

            if scan_buf.is_valid() {
                let name = scan_buf.get_name_string().unwrap();
                write!(
                    debug,
                    "\n*** Password found at 0x{addr:06x}, encr {}+{} bytes, loc: 0x{i:03x} name: {}\n",
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
            writeln!(debug, "Error: no RNG. Open password store first.").ok();
            return None;
        }

        let rand_loc =
            self.rng.as_mut().unwrap().random_range(0..FLASH_SIZE / FLASH_BLOCK_SIZE);
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
        if self.cipher.is_none() {
            writeln!(debug, "Error: password store is not open.").ok();
            return;
        }

        write!(debug, "loc   name\n----------\n").ok();
        for (addr, name) in self {
            writeln!(debug, "0x{:03x} {name}", addr / FLASH_BLOCK_SIZE).ok();
        }
    }

    pub fn search<D>(&mut self, debug: &mut D, srch: &[u8])
    where
        D: Sized + fmt::Write,
    {
        if self.cipher.is_none() {
            writeln!(debug, "Error: password store is not open.").ok();
            return;
        }

        let len = match srch.len() {
            0 => {
                return;
            }
            len => len,
        };

        write!(debug, "loc   name\n----------\n").ok();
        for (addr, name) in self {
            if name.as_bytes().windows(len).any(|window| window == srch) {
                writeln!(debug, "0x{:03x} {name}", addr / FLASH_BLOCK_SIZE).ok();
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
