// climenu.rs

use crate::*;
use crate::spi_memory::prelude::*;
use embedded_cli::Command;

pub const NOECHO_BUF_SIZE: usize = 256;

pub struct AppCtx {
    pub now: u64,
    pub pwd_store: PwdStore,
    pub init: InitState,
    pub open: OpenState,
    pub store: StoreState,
    pub opt_str: Option<String>,
    pub idx1: usize,
    pub buf1: [u8; NOECHO_BUF_SIZE],
    pub idx2: usize,
    pub buf2: [u8; NOECHO_BUF_SIZE],
    pub idx3: usize,
    pub buf3: [u8; NOECHO_BUF_SIZE],
}

#[derive(Debug, Command)]
pub enum AppCommand<'a> {
    /// Show status
    Status,
    /// Initialize password store
    Init,
    /// Open password store
    Open,
    /// Close password store
    Close,
    /// Store secret to flash
    Store {
        name: &'a str,
    },
    /// Fetch secret from flash
    Fetch {
        loc: &'a str,
    },
    /// Drop secret from flash
    Drop {
        loc: &'a str,
    },
    /// Scan flash for secrets
    Scan,
    /// List secrets
    List,
    /// Search secret names
    Search {
        srch: &'a str,
    },
    /// Wait (debug)
    Wait,
    /// Flash operations
    Flash {
        #[command(subcommand)]
        command: FlashCommand<'a>,
    },
}

#[derive(Debug, Command)]
pub enum FlashCommand<'a> {
    /// Read flash memory
    Read {
        addr: &'a str,
        len: Option<&'a str>,
    },
    /// Write flash memory
    Write {
        addr: &'a str,
        len: Option<&'a str>,
        data: Option<&'a str>,
    },
    /// Erase flash memory
    Erase {
        addr: &'a str,
        len: Option<&'a str>,
    },
}

pub fn process_cli_byte(cli: &mut AppCli, app_ctx: &mut AppCtx, byte: u8) {
    let _ = cli.process_byte::<AppCommand<'_>, _>(
        byte,
        &mut AppCommand::processor(|cli_handle, command| {
            dispatch_command(cli_handle.writer(), app_ctx, command);
            Ok(())
        }),
    );
}

fn dispatch_command(w: &mut impl fmt::Write, ctx: &mut AppCtx, command: AppCommand<'_>) {
    match command {
        AppCommand::Status => cmd_status(w, ctx),
        AppCommand::Init => cmd_init(w, ctx),
        AppCommand::Open => cmd_open(w, ctx),
        AppCommand::Close => cmd_close(w, ctx),
        AppCommand::Store { name } => cmd_store(w, ctx, name),
        AppCommand::Fetch { loc } => cmd_fetch(w, ctx, loc),
        AppCommand::Drop { loc } => cmd_drop(w, ctx, loc),
        AppCommand::Scan => cmd_scan(w, ctx),
        AppCommand::List => cmd_list(w, ctx),
        AppCommand::Search { srch } => cmd_search(w, ctx, srch),
        AppCommand::Wait => cmd_wait(w, ctx),
        AppCommand::Flash { command } => dispatch_flash(w, ctx, command),
    }
}

fn dispatch_flash(w: &mut impl fmt::Write, ctx: &mut AppCtx, command: FlashCommand<'_>) {
    match command {
        FlashCommand::Read { addr, len } => cmd_flash_read_write(w, ctx, addr, len, None, false),
        FlashCommand::Write { addr, len, data } => {
            cmd_flash_read_write(w, ctx, addr, len, data, true)
        }
        FlashCommand::Erase { addr, len } => cmd_flash_erase(w, ctx, addr, len),
    }
}

fn cmd_status(w: &mut impl fmt::Write, ctx: &mut AppCtx) {
    ctx.pwd_store.status(w);
}

fn cmd_init(w: &mut impl fmt::Write, ctx: &mut AppCtx) {
    if ctx.pwd_store.is_initialized() {
        write!(
            w,
            "Error: will not overwrite existing master key.\n\
            If you want a fresh start, issue this command:\n\
            flash erase 0 0x800000\n"
        )
        .ok();
        return;
    }
    ctx.idx1 = 0;
    ctx.idx2 = 0;
    ctx.idx3 = 0;
    write!(w, "master password:").ok();
    ctx.init = InitState::AskPass1;
}

fn cmd_open(w: &mut impl fmt::Write, ctx: &mut AppCtx) {
    ctx.idx1 = 0;
    ctx.idx2 = 0;
    ctx.idx3 = 0;
    write!(w, "master password:").ok();
    ctx.open = OpenState::AskPass;
}

fn cmd_close(w: &mut impl fmt::Write, ctx: &mut AppCtx) {
    ctx.pwd_store.close(w);
}

fn cmd_store(w: &mut impl fmt::Write, ctx: &mut AppCtx, name: &str) {
    ctx.opt_str = Some(name.to_string());
    ctx.idx1 = 0;
    ctx.idx2 = 0;
    ctx.idx3 = 0;
    write!(w, "username: ").ok();
    ctx.store = StoreState::AskUser;
}

fn cmd_fetch(w: &mut impl fmt::Write, ctx: &mut AppCtx, loc_s: &str) {
    let loc = if let Some(a) = parse_num(w, loc_s) {
        a
    } else {
        writeln!(w, "Could not parse loc: \"{loc_s}\".").ok();
        return;
    };
    ctx.pwd_store.fetch(w, loc);
}

fn cmd_drop(w: &mut impl fmt::Write, ctx: &mut AppCtx, loc_s: &str) {
    let loc = if let Some(a) = parse_num(w, loc_s) {
        a
    } else {
        writeln!(w, "Could not parse loc: \"{loc_s}\".").ok();
        return;
    };
    ctx.pwd_store.drop(w, loc);
}

fn cmd_scan(w: &mut impl fmt::Write, ctx: &mut AppCtx) {
    ctx.pwd_store.scan(w);
}

fn cmd_list(w: &mut impl fmt::Write, ctx: &mut AppCtx) {
    ctx.pwd_store.list(w);
}

fn cmd_search(w: &mut impl fmt::Write, ctx: &mut AppCtx, srch: &str) {
    ctx.pwd_store.search(w, srch.as_bytes());
}

fn cmd_wait(w: &mut impl fmt::Write, ctx: &mut AppCtx) {
    ctx.idx1 = 0;
    ctx.idx2 = 0;
    ctx.idx3 = 0;
    write!(w, "username: ").ok();
    ctx.store = StoreState::AskUser;
}

fn parse_radix(w: &mut impl fmt::Write, addr: &str, radix: u32) -> Option<usize> {
    match usize::from_str_radix(addr, radix) {
        Err(e) => {
            writeln!(w, "Address (radix {radix}) parse error: {e:?}").ok();
            None
        }
        Ok(a) => Some(a),
    }
}

fn parse_num(w: &mut impl fmt::Write, num: &str) -> Option<usize> {
    if let Some(hex) = num.strip_prefix("0x") {
        parse_radix(w, hex, 16)
    } else if num.starts_with('0') && num.len() > 1 {
        parse_radix(w, &num[1..], 8)
    } else {
        parse_radix(w, num, 10)
    }
}

const HEX_BUF_SZ: usize = 256;

fn cmd_flash_read_write(
    w: &mut impl fmt::Write,
    ctx: &mut AppCtx,
    addr_s: &str,
    len_s: Option<&str>,
    data_s: Option<&str>,
    mode_write: bool,
) {
    let addr = if let Some(a) = parse_num(w, addr_s) {
        a
    } else {
        writeln!(w, "Could not parse addr: \"{addr_s}\".").ok();
        return;
    };
    if addr >= FLASH_SIZE {
        writeln!(
            w,
            "Error: addr {addr} (0x{addr:x}) is larger than flash size {FLASH_SIZE} (0x{FLASH_SIZE:x}).",
        )
        .ok();
        return;
    }

    let mut len = HEX_BUF_SZ;
    if let Some(al) = len_s {
        if let Some(ret) = parse_num(w, al) {
            len = ret;
        } else {
            writeln!(w, "Could not parse len: \"{al}\".").ok();
            return;
        }
    }
    if addr + len > FLASH_SIZE {
        let new_len = FLASH_SIZE - addr;
        writeln!(
            w,
            "Warning: len {len} (0x{len:x}) reaches beyond flash size {FLASH_SIZE} (0x{FLASH_SIZE:x}) \
            and was trucated to {new_len} (0x{new_len:x}).",
        )
        .ok();
        len = new_len;
    }

    let mut fill_byte: Option<u8> = None;
    if let Some(data_str) = data_s {
        if let Some(ret) = parse_num(w, data_str) {
            fill_byte = Some(ret as u8);
        } else {
            writeln!(w, "Could not parse data: \"{data_str}\".").ok();
            return;
        }
    }

    let mut buf = [0u8; HEX_BUF_SZ];
    let mut rng = rand::rngs::StdRng::seed_from_u64(ctx.now);

    let mut chunks = len / HEX_BUF_SZ;
    let last_sz = len % HEX_BUF_SZ;
    if last_sz != 0 {
        chunks += 1;
    }

    write!(
        w,
        "\n* {}ing {len} bytes at 0x{addr:06x} in {chunks} chunks, last {last_sz} bytes:\n",
        if mode_write { "Writ" } else { "Read" },
    )
    .ok();

    for c in 0..chunks {
        let mem_addr = addr + c * HEX_BUF_SZ;
        let mem_len = if last_sz == 0 || c < chunks - 1 {
            HEX_BUF_SZ
        } else {
            last_sz
        };

        if mode_write {
            if let Some(fill_b) = fill_byte {
                buf.iter_mut().for_each(|b| *b = fill_b);
            } else {
                rng.fill_bytes(&mut buf[..mem_len]);
            }

            hex_dump(w, mem_addr, &buf[..mem_len]);
            match ctx
                .pwd_store
                .flash
                .write_bytes(mem_addr as u32, &mut buf[..mem_len])
            {
                Ok(wait) => {
                    writeln!(w, "#wait {wait}").ok();
                }
                Err(e) => {
                    write!(
                        w,
                        "\n### Flash write failed at 0x{mem_addr:06x} ({e:?}) - abort.\n",
                    )
                    .ok();
                    return;
                }
            }
        } else {
            if let Err(e) = ctx
                .pwd_store
                .flash
                .read(mem_addr as u32, &mut buf[..mem_len])
            {
                write!(
                    w,
                    "\n### Flash read failed at 0x{mem_addr:06x} ({e:?}) - abort.\n",
                )
                .ok();
                return;
            }
            hex_dump(w, mem_addr, &buf[..mem_len]);
        }
        // prevent hardware reset during lengthy flash ops
        ctx.pwd_store.watchdog.feed();
    }
    writeln!(w).ok();
}

fn cmd_flash_erase(
    w: &mut impl fmt::Write,
    ctx: &mut AppCtx,
    addr_s: &str,
    len_s: Option<&str>,
) {
    let jedec_id = ctx.pwd_store.flash.read_jedec_id().unwrap();
    write!(w, "\nFlash jedec id: {jedec_id:?}\n").ok();

    let addr = if let Some(a) = parse_num(w, addr_s) {
        a
    } else {
        writeln!(w, "Could not parse addr: \"{addr_s}\".").ok();
        return;
    };
    if addr % FLASH_BLOCK_SIZE != 0 {
        writeln!(
            w,
            "Error: addr {addr} (0x{addr:x}) is not multiple of \
            {FLASH_BLOCK_SIZE} (0x{FLASH_BLOCK_SIZE:02x}).",
        )
        .ok();
        return;
    }
    if addr >= FLASH_SIZE {
        writeln!(
            w,
            "Error: addr {addr} (0x{addr:x}) is larger than flash size \
            {FLASH_SIZE} (0x{FLASH_SIZE:x}).",
        )
        .ok();
        return;
    }

    let mut len = FLASH_BLOCK_SIZE;
    if let Some(al) = len_s {
        if let Some(ret) = parse_num(w, al) {
            len = ret;
        } else {
            writeln!(w, "Could not parse len: \"{al}\".").ok();
            return;
        }
    }
    if !len.is_multiple_of(FLASH_BLOCK_SIZE) {
        writeln!(
            w,
            "Error: len {len} (0x{len:x}) is not multiple of \
            {FLASH_BLOCK_SIZE} (0x{FLASH_BLOCK_SIZE:x}).",
        )
        .ok();
        return;
    }
    if addr + len > FLASH_SIZE {
        let new_len = FLASH_SIZE - addr;
        writeln!(
            w,
            "Warning: len {len} (0x{len:x}) reaches beyond flash size \
            {FLASH_SIZE} (0x{FLASH_SIZE:x}) and was trucated to \
            {new_len} (0x{new_len:x}).",
        )
        .ok();
        len = new_len;
    }

    let sectors = len / FLASH_BLOCK_SIZE;
    writeln!(
        w,
        "* Erasing {len} (0x{len:x}) bytes at 0x{addr:06x} in {sectors} sectors:",
    )
    .ok();

    for c in 0..sectors {
        let mem_addr = addr + c * FLASH_BLOCK_SIZE;
        match ctx.pwd_store.flash.erase_sectors(mem_addr as u32, 1) {
            Ok(wait) => {
                write!(w, "\r#e 0x{mem_addr:06x} #w {wait}      ").ok();
            }
            Err(e) => {
                write!(
                    w,
                    "\n### Flash erase failed at 0x{mem_addr:06x} ({e:?}) - abort.\n",
                )
                .ok();
                return;
            }
        }
        // prevent hardware reset during lengthy flash ops
        ctx.pwd_store.watchdog.feed();
    }
    write!(w, "\ndone.\n").ok();
}

// EOF
