// lib.rs
#![no_std]

use core::fmt;

pub mod pwdstore;
pub use pwdstore::*;

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
