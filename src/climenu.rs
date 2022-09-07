// climenu.rs

use crate::*;
// use hal::{gpio::*, prelude::*, serial, spi::*};
use spi_memory::prelude::*;

pub const NOECHO_BUF_SIZE: usize = 256;

pub struct MyMenuCtx {
    pub now: u64,
    pub serial: MyUsbSerial,
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

impl fmt::Write for MyMenuCtx {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        s.bytes()
            .try_for_each(|c| {
                if c == b'\n' {
                    self.serial.serial.write(&[b'\r'])?;
                }
                self.serial.serial.write(&[c]).map(|_| ())
            })
            .map_err(|_| fmt::Error)
    }
}

pub const ROOT_MENU: menu::Menu<MyMenuCtx> = menu::Menu {
    label: "root",
    entry: None,
    exit: None,
    items: &[
        &menu::Item {
            command: "status",
            help: Some("Show status"),
            item_type: menu::ItemType::Callback {
                function: cmd_status,
                parameters: &[],
            },
        },
        &menu::Item {
            command: "init",
            help: Some("Initialize password store"),
            item_type: menu::ItemType::Callback {
                function: cmd_init,
                parameters: &[],
            },
        },
        &menu::Item {
            command: "open",
            help: Some("Open password store"),
            item_type: menu::ItemType::Callback {
                function: cmd_open,
                parameters: &[],
            },
        },
        &menu::Item {
            command: "close",
            help: Some("Close password store"),
            item_type: menu::ItemType::Callback {
                function: cmd_close,
                parameters: &[],
            },
        },
        &menu::Item {
            command: "store",
            help: Some("Store secret to flash"),
            item_type: menu::ItemType::Callback {
                function: cmd_store,
                parameters: &[menu::Parameter::Mandatory {
                    parameter_name: "name",
                    help: Some("Name of secret"),
                }],
            },
        },
        &menu::Item {
            command: "fetch",
            help: Some("Fetch secret from flash"),
            item_type: menu::ItemType::Callback {
                function: cmd_fetch,
                parameters: &[menu::Parameter::Mandatory {
                    parameter_name: "loc",
                    help: Some("Location to fetch"),
                }],
            },
        },
        &menu::Item {
            command: "drop",
            help: Some("Drop secret from flash"),
            item_type: menu::ItemType::Callback {
                function: cmd_drop,
                parameters: &[menu::Parameter::Mandatory {
                    parameter_name: "loc",
                    help: Some("Location to drop"),
                }],
            },
        },
        &menu::Item {
            command: "scan",
            help: Some("Scan flash for secrets"),
            item_type: menu::ItemType::Callback {
                function: cmd_scan,
                parameters: &[],
            },
        },
        &menu::Item {
            command: "list",
            help: Some("List secrets"),
            item_type: menu::ItemType::Callback {
                function: cmd_list,
                parameters: &[],
            },
        },
        &menu::Item {
            command: "search",
            help: Some("Search secret names with a string"),
            item_type: menu::ItemType::Callback {
                function: cmd_search,
                parameters: &[menu::Parameter::Mandatory {
                    parameter_name: "srch",
                    help: Some("String to search"),
                }],
            },
        },
        &menu::Item {
            command: "wait",
            help: Some("List secrets"),
            item_type: menu::ItemType::Callback {
                function: cmd_wait,
                parameters: &[],
            },
        },
        &menu::Item {
            command: "flash",
            help: Some("Flash functions"),
            item_type: menu::ItemType::Menu(&menu::Menu {
                label: "flash",
                entry: None,
                exit: None,
                items: &[
                    &menu::Item {
                        command: "read",
                        help: Some("Read flash mem"),
                        item_type: menu::ItemType::Callback {
                            function: cmd_flash_read_write,
                            parameters: &[
                                menu::Parameter::Mandatory {
                                    parameter_name: "addr",
                                    help: Some("Start address of flash read"),
                                },
                                menu::Parameter::Optional {
                                    parameter_name: "len",
                                    help: Some("Read length in bytes"),
                                },
                            ],
                        },
                    },
                    &menu::Item {
                        command: "write",
                        help: Some("Write flash mem"),
                        item_type: menu::ItemType::Callback {
                            function: cmd_flash_read_write,
                            parameters: &[
                                menu::Parameter::Mandatory {
                                    parameter_name: "addr",
                                    help: Some("Start address of flash write"),
                                },
                                menu::Parameter::Optional {
                                    parameter_name: "len",
                                    help: Some("Write length in bytes"),
                                },
                                menu::Parameter::Optional {
                                    parameter_name: "data",
                                    help: Some("Fill byte, default is randomized"),
                                },
                            ],
                        },
                    },
                    &menu::Item {
                        command: "erase",
                        help: Some("Erase flash mem"),
                        item_type: menu::ItemType::Callback {
                            function: cmd_flash_erase,
                            parameters: &[
                                menu::Parameter::Mandatory {
                                    parameter_name: "addr",
                                    help: Some(
                                        "Address of flash erase - must be multiple of 4KB (0x1000)",
                                    ),
                                },
                                menu::Parameter::Optional {
                                    parameter_name: "len",
                                    help: Some(
                                        "Length of flash erase - must be multiple of 4KB (0x1000)",
                                    ),
                                },
                            ],
                        },
                    },
                ],
            }),
        },
    ],
};

fn cmd_status(
    _menu: &menu::Menu<MyMenuCtx>,
    _item: &menu::Item<MyMenuCtx>,
    _args: &[&str],
    ctx: &mut MyMenuCtx,
) {
    let pws = &mut ctx.pwd_store;
    pws.status(&mut ctx.serial);
}

fn cmd_init(
    _menu: &menu::Menu<MyMenuCtx>,
    _item: &menu::Item<MyMenuCtx>,
    _args: &[&str],
    ctx: &mut MyMenuCtx,
) {
    if ctx.pwd_store.is_initialized() {
        write!(
            ctx,
            "Error: will not overwrite existing master key.\r\n\
            If you want a fresh start, issue these commands from main menu:\r\n\
            flash\r\n\
            erase 0 0x800000\r\n\
            exit\r\n"
        )
        .ok();
        return;
    }
    ctx.idx1 = 0;
    ctx.idx2 = 0;
    ctx.idx3 = 0;
    write!(ctx, "master password:").ok();
    ctx.init = InitState::AskPass1;
}

fn cmd_open(
    _menu: &menu::Menu<MyMenuCtx>,
    _item: &menu::Item<MyMenuCtx>,
    _args: &[&str],
    ctx: &mut MyMenuCtx,
) {
    ctx.idx1 = 0;
    ctx.idx2 = 0;
    ctx.idx3 = 0;
    write!(ctx, "master password:").ok();
    ctx.open = OpenState::AskPass;
}

fn cmd_close(
    _menu: &menu::Menu<MyMenuCtx>,
    _item: &menu::Item<MyMenuCtx>,
    _args: &[&str],
    ctx: &mut MyMenuCtx,
) {
    let pws = &mut ctx.pwd_store;
    pws.close(&mut ctx.serial);
}

fn cmd_store(
    _menu: &menu::Menu<MyMenuCtx>,
    item: &menu::Item<MyMenuCtx>,
    args: &[&str],
    ctx: &mut MyMenuCtx,
) {
    let name = if let Ok(Some(aa)) = menu::argument_finder(item, args, "name") {
        aa
    } else {
        write!(ctx, "Name not given.\r\n").ok();
        return;
    };

    ctx.opt_str = Some(name.to_string());
    ctx.idx1 = 0;
    ctx.idx2 = 0;
    ctx.idx3 = 0;
    write!(ctx, "username: ").ok();
    ctx.store = StoreState::AskUser;
}

fn cmd_fetch(
    _menu: &menu::Menu<MyMenuCtx>,
    item: &menu::Item<MyMenuCtx>,
    args: &[&str],
    ctx: &mut MyMenuCtx,
) {
    let loc = if let Ok(Some(aa)) = menu::argument_finder(item, args, "loc") {
        if let Some(a) = parse_num(ctx, aa) {
            a
        } else {
            write!(ctx, "Could not parse loc: \"{aa}\".\r\n").ok();
            return;
        }
    } else {
        write!(ctx, "Location not given.\r\n").ok();
        return;
    };

    let pws = &mut ctx.pwd_store;
    pws.fetch(&mut ctx.serial, loc);
}

fn cmd_drop(
    _menu: &menu::Menu<MyMenuCtx>,
    item: &menu::Item<MyMenuCtx>,
    args: &[&str],
    ctx: &mut MyMenuCtx,
) {
    let loc = if let Ok(Some(aa)) = menu::argument_finder(item, args, "loc") {
        if let Some(a) = parse_num(ctx, aa) {
            a
        } else {
            write!(ctx, "Could not parse loc: \"{aa}\".\r\n").ok();
            return;
        }
    } else {
        write!(ctx, "Location not given.\r\n").ok();
        return;
    };

    let pws = &mut ctx.pwd_store;
    pws.drop(&mut ctx.serial, loc);
}

fn cmd_scan(
    _menu: &menu::Menu<MyMenuCtx>,
    _item: &menu::Item<MyMenuCtx>,
    _args: &[&str],
    ctx: &mut MyMenuCtx,
) {
    let pws = &mut ctx.pwd_store;
    pws.scan(&mut ctx.serial);
}

fn cmd_list(
    _menu: &menu::Menu<MyMenuCtx>,
    _item: &menu::Item<MyMenuCtx>,
    _args: &[&str],
    ctx: &mut MyMenuCtx,
) {
    let pws = &mut ctx.pwd_store;
    pws.list(&mut ctx.serial);
}

fn cmd_search(
    _menu: &menu::Menu<MyMenuCtx>,
    item: &menu::Item<MyMenuCtx>,
    args: &[&str],
    ctx: &mut MyMenuCtx,
) {
    let srch = if let Ok(Some(term)) = menu::argument_finder(item, args, "srch") {
        term
    } else {
        write!(ctx, "Search string not given.\r\n").ok();
        return;
    };

    let pws = &mut ctx.pwd_store;
    pws.search(&mut ctx.serial, srch.as_bytes());
}

fn cmd_wait(
    _menu: &menu::Menu<MyMenuCtx>,
    _item: &menu::Item<MyMenuCtx>,
    _args: &[&str],
    ctx: &mut MyMenuCtx,
) {
    ctx.idx1 = 0;
    ctx.idx2 = 0;
    ctx.idx3 = 0;
    write!(ctx, "username: ").ok();
    ctx.store = StoreState::AskUser;
}

fn parse_radix(ctx: &mut MyMenuCtx, addr: &str, radix: u32) -> Option<usize> {
    match usize::from_str_radix(addr, radix) {
        Err(e) => {
            write!(ctx, "Address (radix {radix}) parse error: {e:?}\r\n").ok();
            None
        }
        Ok(a) => Some(a),
    }
}

fn parse_num(ctx: &mut MyMenuCtx, num: &str) -> Option<usize> {
    if let Some(hex) = num.strip_prefix("0x") {
        // Hex parse
        parse_radix(ctx, hex, 16)
    } else if num.starts_with('0') && num.len() > 1 {
        // Octal parse
        parse_radix(ctx, &num[1..], 8)
    } else {
        // Decimal parse
        parse_radix(ctx, num, 10)
    }
}

const HEX_BUF_SZ: usize = 256;
fn cmd_flash_read_write(
    _menu: &menu::Menu<MyMenuCtx>,
    item: &menu::Item<MyMenuCtx>,
    args: &[&str],
    ctx: &mut MyMenuCtx,
) {
    let mode_write = item.command == "write";

    let addr = if let Ok(Some(aa)) = menu::argument_finder(item, args, "addr") {
        if let Some(a) = parse_num(ctx, aa) {
            a
        } else {
            write!(ctx, "Could not parse addr: \"{aa}\".\r\n").ok();
            return;
        }
    } else {
        write!(ctx, "Address not given.\r\n").ok();
        return;
    };
    if addr >= FLASH_SIZE {
        write!(
            ctx,
            "Error: addr {addr} (0x{addr:x}) is larger than flash size {FLASH_SIZE} (0x{FLASH_SIZE:x}).\r\n",
        )
        .ok();
        return;
    }

    let mut len = HEX_BUF_SZ;
    if let Ok(Some(al)) = menu::argument_finder(item, args, "len") {
        if let Some(ret) = parse_num(ctx, al) {
            len = ret;
        } else {
            write!(ctx, "Could not parse len: \"{al}\".\r\n").ok();
            return;
        }
    }
    if addr + len > FLASH_SIZE {
        let new_len = FLASH_SIZE - addr;
        write!(
            ctx,
            "Warning: len {len} (0x{len:x}) reaches beyond flash size {FLASH_SIZE} (0x{FLASH_SIZE:x}) \
            and was trucated to {new_len} (0x{new_len:x}).\r\n",
        )
        .ok();
        len = new_len;
    }

    let mut fill_byte: Option<u8> = None;
    if let Ok(Some(data_s)) = menu::argument_finder(item, args, "data") {
        if let Some(ret) = parse_num(ctx, data_s) {
            fill_byte = Some(ret as u8);
        } else {
            write!(ctx, "Could not parse data: \"{data_s}\".\r\n").ok();
            return;
        }
    }

    let mut buf = [0u8; HEX_BUF_SZ as usize];
    let mut rng = rand::rngs::StdRng::seed_from_u64(ctx.now);

    let mut chunks = len / HEX_BUF_SZ;
    let last_sz = len % HEX_BUF_SZ;
    if last_sz != 0 {
        chunks += 1;
    }

    write!(
        ctx,
        "\r\n* {}ing {len} bytes at 0x{addr:06x} in {chunks} chunks, last {last_sz} bytes:\r\n",
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
                // data byte to use for filling was given
                buf.iter_mut().for_each(|b| *b = fill_b);
            } else {
                // fill buf with random bytes
                rng.fill_bytes(&mut buf[..mem_len]);
            }

            hex_dump(ctx, mem_addr, &buf[..mem_len]);
            match ctx
                .pwd_store
                .flash
                .write_bytes(mem_addr as u32, &mut buf[..mem_len])
            {
                Ok(w) => {
                    write!(ctx, "#wait {w}\r\n").ok();
                }
                Err(e) => {
                    write!(
                        ctx,
                        "\r\n### Flash write failed at 0x{mem_addr:06x} ({e:?}) - abort.\r\n",
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
                    ctx,
                    "\r\n### Flash read failed at 0x{mem_addr:06x} ({e:?}) - abort.\r\n",
                )
                .ok();
                return;
            }
            hex_dump(ctx, mem_addr, &buf[..mem_len]);
        }
        // prevent hardware reset during lengthy flash ops
        ctx.pwd_store.watchdog.feed();
    }
    write!(ctx, "\r\n").ok();
}

fn cmd_flash_erase(
    _menu: &menu::Menu<MyMenuCtx>,
    item: &menu::Item<MyMenuCtx>,
    args: &[&str],
    ctx: &mut MyMenuCtx,
) {
    let jedec_id = ctx.pwd_store.flash.read_jedec_id().unwrap();
    write!(ctx, "\r\nFlash jedec id: {jedec_id:?}\r\n").ok();

    let addr = if let Ok(Some(aa)) = menu::argument_finder(item, args, "addr") {
        if let Some(a) = parse_num(ctx, aa) {
            a
        } else {
            write!(ctx, "Could not parse addr: \"{aa}\".\r\n").ok();
            return;
        }
    } else {
        write!(ctx, "Address not given.\r\n").ok();
        return;
    };
    if addr % FLASH_BLOCK_SIZE != 0 {
        write!(
            ctx,
            "Error: addr {addr} (0x{addr:x}) is not multiple of \
            {FLASH_BLOCK_SIZE} (0x{FLASH_BLOCK_SIZE:02x}).\r\n",
        )
        .ok();
        return;
    }
    if addr >= FLASH_SIZE {
        write!(
            ctx,
            "Error: addr {addr} (0x{addr:x}) is larger than flash size \
            {FLASH_SIZE} (0x{FLASH_SIZE:x}).\r\n",
        )
        .ok();
        return;
    }

    let mut len = FLASH_BLOCK_SIZE;
    if let Ok(Some(al)) = menu::argument_finder(item, args, "len") {
        if let Some(ret) = parse_num(ctx, al) {
            len = ret;
        } else {
            write!(ctx, "Could not parse len: \"{al}\".\r\n").ok();
            return;
        }
    }
    if len % FLASH_BLOCK_SIZE != 0 {
        write!(
            ctx,
            "Error: len {len} (0x{len:x}) is not multiple of \
            {FLASH_BLOCK_SIZE} (0x{FLASH_BLOCK_SIZE:x}).\r\n",
        )
        .ok();
        return;
    }
    if addr + len > FLASH_SIZE {
        let new_len = FLASH_SIZE - addr;
        write!(
            ctx,
            "Warning: len {len} (0x{len:x}) reaches beyond flash size \
            {FLASH_SIZE} (0x{FLASH_SIZE:x}) and was trucated to \
            {new_len} (0x{new_len:x}).\r\n",
        )
        .ok();
        len = new_len;
    }

    let sectors = len / FLASH_BLOCK_SIZE;
    write!(
        ctx,
        "* Erasing {len} (0x{len:x}) bytes at 0x{addr:06x} in {sectors} sectors:\r\n",
    )
    .ok();

    for c in 0..sectors {
        let mem_addr = addr + c * FLASH_BLOCK_SIZE;
        match ctx.pwd_store.flash.erase_sectors(mem_addr as u32, 1) {
            Ok(w) => {
                write!(ctx, "\r#e 0x{mem_addr:06x} #w {w}      ").ok();
            }
            Err(e) => {
                write!(
                    ctx,
                    "\r\n### Flash erase failed at 0x{mem_addr:06x} ({e:?}) - abort.\r\n",
                )
                .ok();
                return;
            }
        }
        // prevent hardware reset during lengthy flash ops
        ctx.pwd_store.watchdog.feed();
    }
    write!(ctx, "\r\ndone.\r\n").ok();
}

// EOF
