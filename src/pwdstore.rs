// pwdstore.rs

extern crate alloc;
extern crate no_std_compat as std;
use std::prelude::v1::*;

use crate::*;

use alloc::string::String;
use alloc::vec::Vec;
use core::fmt;
use privatebox::PrivateBox;
use rand::{prelude::*, SeedableRng};
use sha2::{Digest, Sha256};
use spi_memory::{series25::Flash, BlockDevice, Read};
use std::str;

use stm32f4xx_hal as hal;

use hal::pac::SPI1;
use hal::{gpio::*, spi::*};

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

const KEY: [u8; 32] = [
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xa0, 0xb0, 0xc0, 0xd0, 0xe0, 0xf1,
];
const ADDR: u32 = 0x042000;
const ADDR_MASTER_KEY: u32 = 0x000000;
const FLASH_BLOCK_SIZE: usize = 4096;
const PWD_MAX_LEN: usize = FLASH_BLOCK_SIZE - 2;

// make it exactly the block size
#[repr(C)]
pub struct PwdRepr {
    pub len: u16,
    pub data: [u8; PWD_MAX_LEN],
}

impl PwdRepr {
    pub fn new(data: &[u8]) -> Self {
        let mut len = data.len();
        if len > PWD_MAX_LEN {
            // brutally truncate the data
            len = PWD_MAX_LEN;
        }

        let mut databuf = [0; PWD_MAX_LEN];
        data[..len].iter().enumerate().for_each(|(i, b)| {
            databuf[i] = *b;
        });

        Self {
            len: len as u16,
            data: databuf,
        }
    }

    #[inline(always)]
    pub fn is_valid(&self) -> bool {
        self.len > 0 && self.len < PWD_MAX_LEN as u16
    }

    #[inline(always)]
    pub fn bytes(&self) -> &[u8] {
        let p_bytes;
        unsafe {
            p_bytes = ::core::slice::from_raw_parts(
                (self as *const PwdRepr) as *const u8,
                FLASH_BLOCK_SIZE,
            );
        }
        p_bytes
    }

    #[inline(always)]
    pub fn bytes_mut(&mut self) -> &mut [u8] {
        let p_bytes;
        unsafe {
            p_bytes = ::core::slice::from_raw_parts_mut(
                (self as *mut PwdRepr) as *mut u8,
                FLASH_BLOCK_SIZE,
            );
        }
        p_bytes
    }

    #[inline(always)]
    pub fn get_data(&self) -> Option<&[u8]> {
        if self.is_valid() {
            Some(&self.data[0..self.len as usize])
        } else {
            None
        }
    }
}

pub struct PwdStore {
    pub flash: MyFlash,
    blocks: usize,
    pbox: Option<PrivateBox<StdRng>>,
    bitmap: Vec<u32>,
}

impl PwdStore {
    pub fn new(flash: MyFlash, size: usize) -> Self {
        let bitmap_size = size / (FLASH_BLOCK_SIZE * 32);
        let mut bitmap = Vec::with_capacity(bitmap_size);
        (0..bitmap_size).for_each(|_| bitmap.push(0));
        Self {
            flash,
            blocks: size / FLASH_BLOCK_SIZE,
            pbox: None,
            bitmap,
        }
    }

    pub fn init<D>(&mut self, debug: &mut D, master_pass: &str, seed: u64)
    where
        D: Sized + fmt::Write,
    {
        // Create a new RNG seeded with master_pass and given seed
        let mut master_seed = [0u8; privatebox::KEY_SIZE];
        let mut seed_hasher = Sha256::new();
        seed_hasher.update(unsafe {
            ::core::slice::from_raw_parts((&seed as *const u64) as *const u8, 8)
        });
        seed_hasher.update(master_pass.as_bytes());
        master_seed.copy_from_slice(seed_hasher.finalize().as_slice());
        let mut rng_master = StdRng::from_seed(master_seed);

        let mut pass_hash = [0; privatebox::KEY_SIZE];
        let mut pass_hasher = Sha256::new();
        pass_hasher.update(master_pass.as_bytes());
        pass_hash.copy_from_slice(pass_hasher.finalize().as_slice());
        write!(debug, "Pass hash is {} bytes:\r\n", pass_hash.len()).ok();
        hex_dump(debug, 0, &pass_hash);

        let mut master_key = [0; privatebox::KEY_SIZE];
        rng_master.fill_bytes(&mut master_key);
        write!(debug, "Master key is {} bytes:\r\n", master_key.len()).ok();
        hex_dump(debug, 0, &master_key);

        let rng_runtime = StdRng::from_rng(&mut rng_master).unwrap();
        let pb_runtime = PrivateBox::new(&master_key, rng_runtime);
        self.pbox = Some(pb_runtime);

        // Encrypt & save the master key on flash using master_pass as encryption key
        let mut pb_master = PrivateBox::new(&pass_hash, rng_master);
        let master_enc = pb_master.encrypt(&master_key, &[], &[]).unwrap();
        drop(pb_master);

        let mut flash_pwd = PwdRepr::new(&master_enc);
        write!(
            debug,
            "Flash PwdRepr len {}, buf {} bytes:\r\n",
            flash_pwd.len,
            flash_pwd.data.len()
        )
        .ok();
        hex_dump(debug, 0, flash_pwd.get_data().unwrap());
        let master_b = flash_pwd.bytes_mut();
        self.flash.erase_sectors(ADDR_MASTER_KEY, 1).ok();
        self.flash.write_bytes(ADDR_MASTER_KEY, master_b).ok();
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

        let mut pwd_buf = PwdRepr::new(&[]);
        self.flash.read(ADDR_MASTER_KEY, pwd_buf.bytes_mut()).ok();
        let pwd_crypted = match pwd_buf.get_data() {
            Some(d) => d,
            None => {
                write!(debug, "No master key saved on flash.\r\n").ok();
                return;
            }
        };

        let master_vec = match pb_master.decrypt(pwd_crypted, &[]) {
            Ok((decr, _auth_h)) => {
                write!(debug, "Decrypted key {} bytes:\r\n", decr.len()).ok();
                hex_dump(debug, 0, &decr);
                decr
            }
            Err(e) => {
                write!(debug, "Decrypt failed: {:?}\r\n", e).ok();
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
        let rng_runtime = StdRng::from_seed(runtime_seed);
        let pb_runtime = PrivateBox::new(&master_key, rng_runtime);
        self.pbox = Some(pb_runtime);
        write!(
            debug,
            "Master key decrypted successfully. Password store opened.\r\n"
        )
        .ok();
    }

    pub fn store<D>(&mut self, debug: &mut D, pwd: &str, seed: u64)
    where
        D: Sized + fmt::Write,
    {
        let rng = StdRng::seed_from_u64(seed);
        // encrypt
        let mut pb = PrivateBox::new(&KEY, rng);
        let cont = pb.encrypt(pwd.as_bytes(), &[], &[]).unwrap();

        write!(debug, "Encrypted into {} bytes:\r\n", cont.len()).ok();
        hex_dump(debug, 0, &cont);
        let pwd = PwdRepr::new(&cont);
        write!(debug, "PwdRepr len: {} (0x{:04x})\r\n", pwd.len, pwd.len).ok();
        let pwd_b = pwd.bytes();

        write!(
            debug,
            "buf len: {} (0x{:04x})\r\n",
            pwd_b.len(),
            pwd_b.len()
        )
        .ok();
        hex_dump(debug, 0, &pwd_b[..256]);

        let mut flash_buf = [0; FLASH_BLOCK_SIZE];
        flash_buf.copy_from_slice(pwd_b);

        self.flash.erase_sectors(ADDR, 1).ok();
        self.flash.write_bytes(ADDR, &mut flash_buf).ok();
    }

    pub fn fetch<D>(&mut self, debug: &mut D)
    where
        D: Sized + fmt::Write,
    {
        let mut pwd_buf = PwdRepr::new(&[]);
        write!(debug, "Reading into PwdRepr...\r\n",).ok();
        self.flash.read(ADDR, pwd_buf.bytes_mut()).ok();

        write!(
            debug,
            "Pwd size: {} (0x{:04x})\r\n",
            pwd_buf.len, pwd_buf.len
        )
        .ok();

        let pwd_crypted = match pwd_buf.get_data() {
            Some(d) => d,
            None => {
                write!(debug, "No password present.\r\n").ok();
                return;
            }
        };
        hex_dump(debug, 0, pwd_crypted);

        // we are not encrypting, so shitty seeding is ok
        let rng = StdRng::seed_from_u64(0);
        let pb = PrivateBox::new(&KEY, rng);

        // decrypt it
        let metadata = [];
        match pb.decrypt(pwd_crypted, &metadata) {
            Ok((dec, _auth_h)) => {
                let secret = String::from_utf8_lossy(&dec);
                write!(debug, "Decrypted data: \"{}\"\r\n", secret.as_ref()).ok();
            }
            Err(e) => {
                write!(debug, "Decrypt failed: {:?}\r\n", e).ok();
            }
        }
    }

    pub fn scan<D>(&mut self, debug: &mut D)
    where
        D: Sized + fmt::Write,
    {
        write!(
            debug,
            "Scanning {} (0x{:x}) blocks for secrets.\r\n",
            self.blocks, self.blocks
        )
        .ok();
        let mut len_buf: u16 = 0;

        let p_bytes;
        unsafe {
            p_bytes = ::core::slice::from_raw_parts_mut((&mut len_buf as *mut u16) as *mut u8, 2);
        }
        write!(
            debug,
            "Reading into p_bytes: {} (0x{:02x}) bytes\r\nScanning flash:\r\n",
            p_bytes.len(),
            p_bytes.len()
        )
        .ok();

        for i in 0..self.blocks {
            let addr = i * FLASH_BLOCK_SIZE;
            self.flash.read(addr as u32, p_bytes).ok();
            if len_buf > 0 && len_buf <= PWD_MAX_LEN as u16 {
                write!(
                    debug,
                    "\r\nPassword found at 0x{:06x}, encrypted len {} bytes\r\n",
                    addr, len_buf
                )
                .ok();
            }
            if i % 32 == 0 {
                write!(debug, ".").ok();
            }
        }
    }
}
// EOF
